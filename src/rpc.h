#ifndef __RPC_H
#define __RPC_H

#define RPC_BUFFER_SIZE 1024

#define RPC_ID_ADC_TEST 0x1
#define RPC_ID_SET_ESC_SPEED 0x2
#define RPC_ID_GET_ESC_SPEED 0x3

struct rpc_request {
    int id;
    int size;
    int pos;
    void *data;
};

void rpc_init();
int rpc_request_get(struct rpc_request *rq);
void rpc_answer_push(void *data, int size);
void rpc_answer_commit(int id);
void rpc_answer_send(int id, int size, void *data);
void rpc_test();

static inline int rpc_pop_int32 ( struct rpc_request *rq )
{
    uint8_t *buf = rq->data + rq->pos;
    int x = 0;

    x|=((int)buf[0])<<24;
    x|=((int)buf[1])<<16;
    x|=((int)buf[2])<<8;
    x|=((int)buf[3])<<0;

    rq->pos += 4;
    return x;
}


#endif

