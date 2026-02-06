CC := gcc
CFLAGS := -O2 -Wall -Wextra -std=c99 -D_XOPEN_SOURCE=500

dma-paced: dma-paced.o mailbox.o
	$(CC) $(CFLAGS) -o $@ $^ -lmdma-paced

main: main.o mailbox.o
	$(CC) $(CFLAGS) -o $@ $^ -lm

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f ./*.o ./dma-paced ./main
