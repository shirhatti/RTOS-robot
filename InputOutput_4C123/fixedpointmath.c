#include <stdint.h>
# define DISTANCEBETWEENSENSORS 13

int16_t array[91] = { 1000, 1000, 999, 999, 998, 996, 995, 993, 990, 988, 985, 982, 978, 974, 970, 966, 961, 956, 951, 946, 940, 934, 927, 921, 914, 906, 899, 891, 883, 875, 866, 857, 848, 839, 829, 819, 809, 799, 788, 777, 766, 755, 743, 731, 719, 707, 695, 682, 669, 656, 643, 629, 616, 602, 588, 574, 559, 545, 530, 515, 500, 485, 469, 454, 438, 423, 407, 391, 375, 358, 342, 326, 309, 292, 276, 259, 242, 225, 208, 191, 174, 156, 139, 122, 105, 87, 70, 52, 35, 17, 0 };

int16_t cosine (int16_t degree) {
	  if(degree<0) degree = -degree;
    return array[degree];
}

int16_t sine(int16_t degree) {
    return array[90-degree];
}

//return value in degree
int32_t arctangent(int32_t q, int32_t i) {
    int32_t degree =  (183328*i*q) / ( ((i*i)<<5) + ((q*q)<<3) + (q*q) ); 
    return (degree+50)/100;
}

int16_t get_angle(int16_t d1, int16_t d2) {
    return arctangent(d2-d1, DISTANCEBETWEENSENSORS);
}

int16_t get_d(int16_t d1, int16_t d2, int16_t angle) {
    return (cosine(angle) * (d1+d2)/2/1000 );
}
