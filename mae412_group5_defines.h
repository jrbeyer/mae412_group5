#ifndef MAE412_H
#define MAE412_H


// REPLACE WITH THE MAC Address of your receiver 
uint8_t baseESPAddress[] = {0x80, 0x65, 0x99, 0x4A, 0x10, 0x08};
uint8_t pixyESPAddress[] = {0x80, 0x65, 0x99, 0x49, 0x64, 0xEC};

//Must match the receiver structure
typedef struct message_S {
    bool      pixy_saw_train; // true if saw a train last frame
    uint16_t  pixy_train_x; // global x coordinates
    uint16_t  pixy_train_y; // global y coordinates
    bool      rangefinder_got_range; //distance detected
    uint16_t  rangefinder_range_mm;
    String    debug;
} struct_message;


#endif //MAE412_H