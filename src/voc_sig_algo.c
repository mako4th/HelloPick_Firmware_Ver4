//30-second tumbling windows.
//センサーデータ取得間隔約5.3s,time window 30s
//A=w0: [head-winWidth, head)
//B=w1: [head-winWidth*2, head-winWidth*1)
//C=w2: [head-winWidth*3, head-winWidth*2)
//D=w3: [head-winWidth*4, head-winWidth*3)
//
//w[winNum] : [head-winWidth*(winNum+1),head-winWidth*winNum)
// 6 samples per window * 4 = 24 samples

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "voc_sig_algo.h"

static struct {
	uint16_t buffer[BUFFER_SIZE];
	uint16_t head;
} voc_buffer;

void voc_buf_reset(void) {
	memset(voc_buffer.buffer, 0, sizeof(voc_buffer.buffer));
	voc_buffer.head = 0;
}

uint16_t buf_push(uint16_t voc) {
	voc_buffer.buffer[voc_buffer.head] = voc;
	voc_buffer.head = (voc_buffer.head + 1) % BUFFER_SIZE;
	return voc;
}

size_t voc_buf_snapshot(uint16_t *out,size_t winWidth,uint16_t winNum){
	for(size_t i=0;i<winWidth;i++){
		int index = ( BUFFER_SIZE + voc_buffer.head - winWidth * (winNum + 1) + i) % BUFFER_SIZE;
		out[i] = voc_buffer.buffer[index];
	}
	return winWidth;
}

static int cmp_u16(const void *pa, const void *pb) {
    uint16_t a = *(const uint16_t*)pa;
    uint16_t b = *(const uint16_t*)pb;
    return (a > b) - (a < b);
}

uint16_t median_u16( const uint16_t *src, size_t n, uint16_t *work_buf, size_t work_buf_size) {
	n = n < work_buf_size ? n : work_buf_size;
    memcpy(work_buf, src, n * sizeof(uint16_t));
    qsort(work_buf, n, sizeof(uint16_t), cmp_u16);
	uint16_t result =(work_buf[(n-1)/2] + work_buf[(n-1)/2+1]) / 2;
	return result;
}

int median_list(uint16_t *out){
	uint16_t tmpbuf[10] = {0};
	uint16_t workbuf[10] = {0};
	
	for(int i=0;i<WINDOW_COUNT;i++){
		voc_buf_snapshot(tmpbuf,WINDOW_WIDTH,i);
		out[i] = median_u16(tmpbuf,WINDOW_WIDTH,workbuf,10);
	}
	return WINDOW_COUNT;
}

static int32_t state_s = 0;
int voc_flag(){
    uint16_t med[4];
    median_list(med);

    // 無効中央値 チェック
	for(int i=0;i<4;i++){
		if(med[i] == 0) return 0; // 判定不可
	}

    int32_t A = (int32_t)med[0];
    int32_t D = (int32_t)med[3];
    // 発生判定: D - A >= 300
	if ((D - A) >= 300) {
		if (state_s == 0) {
			state_s = (A + D) / 2;  // S初期化
			return 2;
		}else{
			return 3;
		}
	}
    // S無効なら正常
    if (state_s == 0) return 1;

    // 正常復帰条件
    if ((A - state_s) >= 300) { state_s = 0; return 1; }
    if ((A - D) >= 100)       { state_s = 0; return 1; }
    return 3; // におい継続
}

int get_state_s(){return state_s;}

//テスト用
//static int test(void){
//	for(int i=0;i<BUFFER_SIZE;i++){
//		buf_push(i*1500);
//	}
//	printf("buffer\n");
//	for(int i=0;i<BUFFER_SIZE;i++){
//		printf("%d ",voc_buffer.buffer[i]);
//	}
//	printf("\n");
//	printf("head = %d\n",voc_buffer.head);
//
//	return 0;
//}
////int main(void){
//	return test();
//}
