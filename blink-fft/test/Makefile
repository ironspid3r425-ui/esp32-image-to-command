# Host build of the blink DSP decoder + Unity tests (no ESP32 needed).
CC      := gcc
CFLAGS  := -std=c99 -Wall -Wextra -Werror -I.. -Ivendor/unity
BUILD   := build

.PHONY: test clean
test: $(BUILD)/test_blink_dsp
	./$(BUILD)/test_blink_dsp

$(BUILD)/test_blink_dsp: ../blink_dsp.c ../blink_dsp.h test_blink_dsp.c vendor/unity/unity.c | $(BUILD)
	$(CC) $(CFLAGS) ../blink_dsp.c vendor/unity/unity.c test_blink_dsp.c -lm -o $@

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
