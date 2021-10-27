
    // clang-format off
/**********Mission structs/Enums***********/

#pragma pack(1)
    typedef struct ServiceAck {
        bool result;
        int cmd_set;
        int cmd_id;
        unsigned int ack_data;

        ServiceAck(bool res, int set, int id, unsigned int ack)
                : result(res), cmd_set(set), cmd_id(id), ack_data(ack) {
        }

        ServiceAck() {
        }
    } ServiceAck;
#pragma pack()