#ifndef Basic_IO_operations
#define Basic_IO_operations

#define set_bit(port, bit) (port |= (1 << bit))
#define clear_bit(port, bit) (port &= ~(1 << bit))
#define check_bit(port, bit) (port & (1 << bit))
#define invert_bit(port, bit) (port ^= (1 << bit))

#endif