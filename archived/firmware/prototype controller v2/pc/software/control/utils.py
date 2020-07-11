def unsigned_to_signed(unsigned_array,N):
    signed = 0
    for i in range(N):
        signed = signed + int(unsigned_array[i])*(256**(N-1-i))
    signed = signed - (256**N)/2
    return signed

def unsigned_to_unsigned(unsigned_array,N):
    unsigned = 0
    for i in range(N):
        unsigned = unsigned + int(unsigned_array[i])*(256**(N-1-i))
    return unsigned