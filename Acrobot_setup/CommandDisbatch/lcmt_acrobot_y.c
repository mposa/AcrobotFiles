// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "lcmt_acrobot_y.h"

static int __lcmt_acrobot_y_hash_computed;
static uint64_t __lcmt_acrobot_y_hash;

uint64_t __lcmt_acrobot_y_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __lcmt_acrobot_y_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__lcmt_acrobot_y_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0x1b1a2a81e14c5a07LL
         + __int64_t_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __lcmt_acrobot_y_get_hash(void)
{
    if (!__lcmt_acrobot_y_hash_computed) {
        __lcmt_acrobot_y_hash = (int64_t)__lcmt_acrobot_y_hash_recursive(NULL);
        __lcmt_acrobot_y_hash_computed = 1;
    }

    return __lcmt_acrobot_y_hash;
}

int __lcmt_acrobot_y_encode_array(void *buf, int offset, int maxlen, const lcmt_acrobot_y *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].timestamp), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].theta1), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].theta2), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].tau), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int lcmt_acrobot_y_encode(void *buf, int offset, int maxlen, const lcmt_acrobot_y *p)
{
    int pos = 0, thislen;
    int64_t hash = __lcmt_acrobot_y_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __lcmt_acrobot_y_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __lcmt_acrobot_y_encoded_array_size(const lcmt_acrobot_y *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int64_t_encoded_array_size(&(p[element].timestamp), 1);

        size += __double_encoded_array_size(&(p[element].theta1), 1);

        size += __double_encoded_array_size(&(p[element].theta2), 1);

        size += __double_encoded_array_size(&(p[element].tau), 1);

    }
    return size;
}

int lcmt_acrobot_y_encoded_size(const lcmt_acrobot_y *p)
{
    return 8 + __lcmt_acrobot_y_encoded_array_size(p, 1);
}

int __lcmt_acrobot_y_decode_array(const void *buf, int offset, int maxlen, lcmt_acrobot_y *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].timestamp), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].theta1), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].theta2), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].tau), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __lcmt_acrobot_y_decode_array_cleanup(lcmt_acrobot_y *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_decode_array_cleanup(&(p[element].timestamp), 1);

        __double_decode_array_cleanup(&(p[element].theta1), 1);

        __double_decode_array_cleanup(&(p[element].theta2), 1);

        __double_decode_array_cleanup(&(p[element].tau), 1);

    }
    return 0;
}

int lcmt_acrobot_y_decode(const void *buf, int offset, int maxlen, lcmt_acrobot_y *p)
{
    int pos = 0, thislen;
    int64_t hash = __lcmt_acrobot_y_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __lcmt_acrobot_y_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int lcmt_acrobot_y_decode_cleanup(lcmt_acrobot_y *p)
{
    return __lcmt_acrobot_y_decode_array_cleanup(p, 1);
}

int __lcmt_acrobot_y_clone_array(const lcmt_acrobot_y *p, lcmt_acrobot_y *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_clone_array(&(p[element].timestamp), &(q[element].timestamp), 1);

        __double_clone_array(&(p[element].theta1), &(q[element].theta1), 1);

        __double_clone_array(&(p[element].theta2), &(q[element].theta2), 1);

        __double_clone_array(&(p[element].tau), &(q[element].tau), 1);

    }
    return 0;
}

lcmt_acrobot_y *lcmt_acrobot_y_copy(const lcmt_acrobot_y *p)
{
    lcmt_acrobot_y *q = (lcmt_acrobot_y*) malloc(sizeof(lcmt_acrobot_y));
    __lcmt_acrobot_y_clone_array(p, q, 1);
    return q;
}

void lcmt_acrobot_y_destroy(lcmt_acrobot_y *p)
{
    __lcmt_acrobot_y_decode_array_cleanup(p, 1);
    free(p);
}

int lcmt_acrobot_y_publish(lcm_t *lc, const char *channel, const lcmt_acrobot_y *p)
{
      int max_data_size = lcmt_acrobot_y_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = lcmt_acrobot_y_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _lcmt_acrobot_y_subscription_t {
    lcmt_acrobot_y_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void lcmt_acrobot_y_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    lcmt_acrobot_y p;
    memset(&p, 0, sizeof(lcmt_acrobot_y));
    status = lcmt_acrobot_y_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding lcmt_acrobot_y!!!\n", status);
        return;
    }

    lcmt_acrobot_y_subscription_t *h = (lcmt_acrobot_y_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    lcmt_acrobot_y_decode_cleanup (&p);
}

lcmt_acrobot_y_subscription_t* lcmt_acrobot_y_subscribe (lcm_t *lcm,
                    const char *channel,
                    lcmt_acrobot_y_handler_t f, void *userdata)
{
    lcmt_acrobot_y_subscription_t *n = (lcmt_acrobot_y_subscription_t*)
                       malloc(sizeof(lcmt_acrobot_y_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 lcmt_acrobot_y_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg lcmt_acrobot_y LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int lcmt_acrobot_y_subscription_set_queue_capacity (lcmt_acrobot_y_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int lcmt_acrobot_y_unsubscribe(lcm_t *lcm, lcmt_acrobot_y_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe lcmt_acrobot_y_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

