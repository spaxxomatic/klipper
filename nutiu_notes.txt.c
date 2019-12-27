// The goal of this code is to take a series of scheduled stepper
// pulse times and compress them into a handful of commands that can
// be efficiently transmitted and executed on a microcontroller (mcu).
// The mcu accepts step pulse commands that take interval, count, and
// add parameters such that 'count' pulses occur, with each step event
// calculating the next step event time using:
//  next_wake_time = last_wake_time + interval; interval += add

//The parameters for each queue_step command are “interval”, “count”, and “add”. At a high-level, stepper_event() runs the following, ‘count’ times: do_step(); next_wake_time = last_wake_time + interval; interval += add;

struct stepcompress {
    // Buffer management
    uint32_t *queue, *queue_end, *queue_pos, *queue_next;
    // Internal tracking
    uint32_t max_error;
    double mcu_time_offset, mcu_freq;
    // Message generation
    uint64_t last_step_clock;
    struct list_head msg_queue;
    uint32_t queue_step_msgid, set_next_step_dir_msgid, oid;
    int sdir, invert_sdir;
};

struct queue_append {
    struct stepcompress *sc;
    uint32_t *qnext, *qend, last_step_clock_32;
    double clock_offset;
};

struct queue_append queue_append_start(
    struct stepcompress *sc, double print_time, double adjust);
    
struct step_move {
    uint32_t interval;
    uint16_t count;
    int16_t add;
};


struct queue_message {
    int len;
    uint8_t msg[MESSAGE_MAX];
    union {
        // Filled when on a command queue
        struct {
            uint64_t min_clock, req_clock;
        };
        // Filled when in sent/receive queues
        struct {
            double sent_time, receive_time;
        };
    };
    struct list_node node;
};