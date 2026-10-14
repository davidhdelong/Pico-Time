/* Host fake-lwIP harness. The runner inserts the actual HTTP implementation. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef int err_t;
typedef uint16_t u16_t;
enum { ERR_OK = 0, ERR_MEM = -1, ERR_ABRT = -2, ERR_CONN = -3 };
#define TCP_MSS 1460
struct pbuf { u16_t tot_len; const char *payload; };
struct tcp_pcb {
    void *callback_arg;
    err_t (*recv)(void *, struct tcp_pcb *, struct pbuf *, err_t);
    err_t (*sent)(void *, struct tcp_pcb *, u16_t);
    err_t (*poll)(void *, struct tcp_pcb *);
    void (*error)(void *, err_t);
    const char *queued[64];
    u16_t lengths[64];
    unsigned count;
    char wire[30000];
    size_t wire_len;
    bool dead;
};
static size_t allocated, peak;
static unsigned allocations, fail_alloc, write_fail, output_fail, close_fail, builds, health_padding;
static u16_t send_space = TCP_MSS;
static uint32_t http_request_count, http_error_count;
/* Deliberately larger than a segment, never copied to heap. */
static const char http_dashboard[9000] = "<!doctype html>test dashboard";
static void *mem_malloc(size_t n) {
    if (fail_alloc) { --fail_alloc; return NULL; }
    size_t *p = malloc(sizeof(size_t) + n);
    assert(p); *p = n; allocated += n; ++allocations;
    if (allocated > peak) peak = allocated;
    return p + 1;
}
static void *mem_calloc(size_t n, size_t s) {
    void *p = mem_malloc(n * s); if (p) memset(p, 0, n * s); return p;
}
static void mem_free(void *p) {
    size_t *base = (size_t *)p - 1; allocated -= *base; free(base);
}
static void *mem_trim(void *p, size_t n) {
    size_t *base = (size_t *)p - 1;
    assert(n <= *base); allocated -= *base - n;
    base = realloc(base, sizeof(size_t) + n); assert(base); *base = n;
    return base + 1;
}
static void tcp_arg(struct tcp_pcb *p, void *arg) { assert(!p->dead); p->callback_arg = arg; }
static void tcp_recv(struct tcp_pcb *p, err_t (*f)(void *, struct tcp_pcb *, struct pbuf *, err_t)) { assert(!p->dead); p->recv = f; }
static void tcp_sent(struct tcp_pcb *p, err_t (*f)(void *, struct tcp_pcb *, u16_t)) { assert(!p->dead); p->sent = f; }
static void tcp_err(struct tcp_pcb *p, void (*f)(void *, err_t)) { assert(!p->dead); p->error = f; }
static void tcp_poll(struct tcp_pcb *p, err_t (*f)(void *, struct tcp_pcb *), unsigned interval) { assert(!p->dead); (void)interval; p->poll = f; }
static err_t tcp_close(struct tcp_pcb *p) {
    if (close_fail) { --close_fail; return ERR_MEM; }
    assert(!p->count); p->dead = true; return ERR_OK;
}
static void tcp_abort(struct tcp_pcb *p) {
    assert(!p->dead); p->dead = true; p->count = 0;
    if (p->error) p->error(p->callback_arg, ERR_ABRT);
    p->callback_arg = NULL;
}
static u16_t tcp_sndbuf(struct tcp_pcb *p) { (void)p; return send_space; }
static err_t tcp_write(struct tcp_pcb *p, const void *data, u16_t len, unsigned flags) {
    assert(!p->dead); assert(flags == 0); assert(p->count < 64);
    if (write_fail) { --write_fail; return ERR_MEM; }
    p->queued[p->count] = data; p->lengths[p->count++] = len; return ERR_OK;
}
static err_t tcp_output(struct tcp_pcb *p) { (void)p; if (output_fail) { --output_fail; return ERR_MEM; } return ERR_OK; }
static void tcp_recved(struct tcp_pcb *p, u16_t len) { (void)p; (void)len; }
static void pbuf_free(struct pbuf *p) { (void)p; }
static uint8_t pbuf_get_at(struct pbuf *p, u16_t i) { return (uint8_t)p->payload[i]; }
static void pbuf_copy_partial(struct pbuf *p, void *out, u16_t n, u16_t offset) { memcpy(out, p->payload + offset, n); }
static int http_build_health_json(char *out, size_t cap) {
    ++builds;
    if (health_padding) {
        assert(cap > health_padding && health_padding > 20);
        int n = snprintf(out, cap, "{\"snapshot\":%u,\"pad\":\"", builds);
        memset(out + n, 'x', health_padding - (size_t)n - 2);
        memcpy(out + health_padding - 2, "\"}", 3);
        return (int)health_padding;
    }
    return snprintf(out, cap, "{\"snapshot\":%u}", builds);
}

/* ACTUAL_HTTP_IMPLEMENTATION */

static err_t feed(struct tcp_pcb *p, const char *s) {
    struct pbuf b = { (u16_t)strlen(s), s };
    return p->recv(p->callback_arg, p, &b, ERR_OK);
}
static void ack(struct tcp_pcb *p, unsigned n) {
    assert(p->count && n <= p->lengths[0]);
    memcpy(p->wire + p->wire_len, p->queued[0], n); p->wire_len += n;
    p->queued[0] += n; p->lengths[0] -= n;
    if (!p->lengths[0]) {
        --p->count;
        memmove(p->queued, p->queued + 1, p->count * sizeof(p->queued[0]));
        memmove(p->lengths, p->lengths + 1, p->count * sizeof(p->lengths[0]));
    }
    assert(p->sent);
    err_t err = p->sent(p->callback_arg, p, (u16_t)n);
    assert(err == ERR_OK || err == ERR_ABRT);
}
static void drain(struct tcp_pcb *p) {
    for (unsigned i = 0; !p->dead && i < 30000; ++i) {
        if (p->count) ack(p, p->lengths[0] > 127 ? 127 : p->lengths[0]);
        else assert(p->poll(p->callback_arg, p) == ERR_OK);
    }
    assert(p->dead);
}
static void expect_body(struct tcp_pcb *p, const char *body, size_t len) {
    const char *start = strstr(p->wire, "\r\n\r\n"); assert(start); start += 4;
    assert(p->wire_len - (size_t)(start - p->wire) == len);
    assert(!memcmp(start, body, len));
    char length[80]; snprintf(length, sizeof(length), "Content-Length: %zu\r\n", len);
    assert(strstr(p->wire, length));
}
int main(void) {
    struct tcp_pcb p = {0}, q = {0}, rejected = {0};
    assert(http_accept(NULL, &p, ERR_OK) == ERR_OK);
    /* Headers fragmented at every byte; response starts exactly once. */
    const char *req = "GET / HTTP/1.1\r\nHost: clock\r\nConnection: close\r\n\r\n";
    for (size_t i = 0; req[i]; ++i) { char c[2] = {req[i], 0}; assert(feed(&p, c) == ERR_OK); }
    assert(http_request_count == 1 && allocations == 1);
    assert(feed(&p, "GET /api/health HTTP/1.1\r\n\r\n") == ERR_OK);
    assert(http_request_count == 1 && builds == 0);
    /* A peer half-close must not free a pending no-copy response. */
    assert(p.recv(p.callback_arg, &p, NULL, ERR_OK) == ERR_OK);
    close_fail = 1;
    drain(&p); expect_body(&p, http_dashboard, sizeof(http_dashboard)-1);
    assert(!allocated && !http_active_connections);
    /* Independent snapshots, with two bounded live connections. */
    memset(&p, 0, sizeof(p));
    assert(http_accept(NULL, &p, ERR_OK) == ERR_OK);
    assert(http_accept(NULL, &q, ERR_OK) == ERR_OK);
    assert(http_accept(NULL, &rejected, ERR_OK) == ERR_ABRT && rejected.dead);
    write_fail = 1;
    assert(feed(&p, "GET /api/health HTTP/1.1\r\n\r\n") == ERR_OK);
    assert(!p.count); output_fail = 1;
    assert(p.poll(p.callback_arg, &p) == ERR_OK);
    assert(feed(&q, "GET /api/health HTTP/1.1\r\n\r\n") == ERR_OK);
    assert(builds == 2);
    drain(&q); expect_body(&q, "{\"snapshot\":2}", 14);
    drain(&p); expect_body(&p, "{\"snapshot\":1}", 14);
    assert(!allocated && !http_active_connections);
    /* Near-capacity health bodies retained simultaneously, with ACKs only
     * after every segment has been queued. Never close on a partial ACK. */
    health_padding = 6000;
    memset(&p, 0, sizeof(p)); memset(&q, 0, sizeof(q));
    assert(http_accept(NULL, &p, ERR_OK) == ERR_OK);
    assert(http_accept(NULL, &q, ERR_OK) == ERR_OK);
    assert(feed(&p, "GET /api/health HTTP/1.1\r\n\r\n") == ERR_OK);
    assert(feed(&q, "GET /api/health HTTP/1.1\r\n\r\n") == ERR_OK);
    char expected_p[6001], expected_q[6001];
    memcpy(expected_p, ((http_conn_state_t *)p.callback_arg)->response, sizeof(expected_p));
    memcpy(expected_q, ((http_conn_state_t *)q.callback_arg)->response, sizeof(expected_q));
    for (unsigned i = 0; i < 6; ++i) assert(p.poll(p.callback_arg, &p) == ERR_OK);
    http_conn_state_t *state = p.callback_arg;
    assert(state->response_offset == state->response_len && !state->acknowledged);
    ack(&p, 1); assert(!p.dead && p.callback_arg == state);
    drain(&q); drain(&p);
    expect_body(&q, expected_q, 6000); expect_body(&p, expected_p, 6000);
    assert(!allocated && !http_active_connections); health_padding = 0;
    /* No sndbuf space, retried from poll rather than stranded forever. */
    memset(&p, 0, sizeof(p)); assert(http_accept(NULL, &p, ERR_OK) == ERR_OK);
    send_space = 0; assert(feed(&p, "GET /missing HTTP/1.1\r\n\r\n") == ERR_OK);
    send_space = 7; drain(&p); expect_body(&p, "Not found\n", 10); send_space = TCP_MSS;
    /* Oversized request line, oversized headers, idle timeout, aborted peer. */
    for (unsigned test = 0; test < 4; ++test) {
        memset(&p, 0, sizeof(p)); assert(http_accept(NULL, &p, ERR_OK) == ERR_OK);
        if (test == 0) {
            char big[200]; memset(big, 'x', sizeof(big)-1); big[199] = 0;
            assert(feed(&p, big) == ERR_OK); drain(&p); assert(strstr(p.wire, "400 Bad Request"));
        } else if (test == 1) {
            assert(feed(&p, "GET / HTTP/1.1\r\n") == ERR_OK);
            char big[4097]; memset(big, 'x', sizeof(big)-1); big[4096] = 0;
            assert(feed(&p, big) == ERR_OK); drain(&p); assert(strstr(p.wire, "400 Bad Request"));
        } else if (test == 2) {
            for (unsigned i = 0; i < 15; ++i) {
                err_t e = p.poll(p.callback_arg, &p); assert(e == (i == 14 ? ERR_ABRT : ERR_OK));
            }
        } else {
            assert(feed(&p, "GET /api/health HTTP/1.1\r\n\r\n") == ERR_OK); tcp_abort(&p);
        }
        assert(!allocated && !http_active_connections);
    }
    memset(&p, 0, sizeof(p)); fail_alloc = 1;
    assert(http_accept(NULL, &p, ERR_OK) == ERR_ABRT && p.dead);
    memset(&p, 0, sizeof(p)); assert(http_accept(NULL, &p, ERR_OK) == ERR_OK);
    fail_alloc = 1;
    assert(feed(&p, "GET /api/health HTTP/1.1\r\n\r\n") == ERR_ABRT);
    assert(!allocated && !http_active_connections);
    assert(peak <= HTTP_MAX_CONNECTIONS * (sizeof(http_conn_state_t) + HTTP_HEALTH_CAPACITY));
    printf("HTTP lifecycle tests passed; host state=%zu, peak app heap=%zu bytes\n", sizeof(http_conn_state_t), peak);
}
