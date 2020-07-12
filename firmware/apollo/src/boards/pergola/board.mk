#
# Build specifics for Pergola hardware.
#

# This is an external board, so its identity is determined by its revision number.
# MAJOR = external board
# MINOR = 0 (Daisho)
# MINOR = 1 (Pergola)
BOARD_REVISION_MAJOR := 255
BOARD_REVISION_MINOR := 1

APP_START_ADDRESS ?= 0x6000C000

flash-hf2: $(BUILD)/$(BOARD)-firmware.bin
	RUST_LOG=debug RUST_BACKTRACE=full hf2 -v 0x239a -p 0x0058 flash --address $(APP_START_ADDRESS) --file $^
