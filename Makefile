CFLAGS=-O2 -Wall

all: gs232_star_tracker

gs232_star_tracker: tracker.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f gs232_star_tracker
        