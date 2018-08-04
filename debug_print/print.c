//gcc -o print print.c
//cp print /tmp/.
#include <stdio.h>
#include <unistd.h>

void crlfcut(char *buf);

int main(int argc, char **argv)
{
	FILE *fp;
	char tmp[1024];

	if((fp = fopen("/tmp/test.log", "r")) == NULL) {
		printf("fopen err\n");
		return -1;
	}

	printf("\033[2J");		//画面をクリア
	printf("\033[33m");		//文字を黄色に
	printf("\033[>5h");		//カーソル消去

	for(;;){
		usleep(50000);	//50ms
		if(fgets(tmp,1023 ,fp ) == 0){
			fseek(fp,  0L, SEEK_SET);
			continue;
		}
		crlfcut(tmp);
		printf(tmp);
		printf("\r");
		fflush(stdout);			//バッファをフラッシュ
	}

	fclose(fp);
}


void crlfcut(char *buf)
{
	int i;

	for(i=0;;i++){
		if(buf[i] == 0)return;
		if(buf[i] == 0x0d || buf[i] == 0x0a){
			buf[i]=0;
			return;
		}
	}
}
