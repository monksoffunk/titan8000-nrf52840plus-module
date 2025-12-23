KEYBOARD_NAME		:= titan8000
SHIELD_STR			:="$(KEYBOARD_NAME) "
DEFAULT_TARGET_DIR 	:= build/$(KEYBOARD_NAME)/zephyr
DEFAULT_TARGET		:= $(DEFAULT_TARGET_DIR)/zmk.uf2
BOARD1				:= xiao_ble_plus
BOARD2				:= xiao_rp2040
TARGET1				:= $(KEYBOARD_NAME)_$(BOARD1).uf2
TARGET2				:= $(KEYBOARD_NAME)_$(BOARD2).uf2

all	: $(TARGET1) $(TARGET2) 

$(TARGET1):
	west build -s zmk/app -b seeeduino_xiao_ble -d build/titan8000 -p always -- -DSHIELD=titan8000
	mv $(DEFAULT_TARGET) $(TARGET1)

$(TARGET2):
	west build -s zmk/app -b seeeduino_xiao_rp2040 -d build/titan8000 -p always -- -DSHIELD=titan8000
	mv $(DEFAULT_TARGET) $(TARGET2)

clean:
	@rm -rf build

fclean: clean
	@rm -f $(TARGET1) $(TARGET2)

re: fclean all

.PHONY: all re clean fclean