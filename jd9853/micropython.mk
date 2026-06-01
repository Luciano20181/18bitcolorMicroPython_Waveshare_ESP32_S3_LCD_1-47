JD9853_MOD_DIR := $(USERMOD_DIR)

CFLAGS_USERMOD += -I$(JD9853_MOD_DIR)

SRC_USERMOD += $(addprefix $(JD9853_MOD_DIR)/, jd9853.c)
SRC_USERMOD += $(addprefix $(JD9853_MOD_DIR)/, mpfile.c)
SRC_USERMOD += $(addprefix $(JD9853_MOD_DIR)/jpg/, tjpgd666.c)
SRC_USERMOD += $(addprefix $(JD9853_MOD_DIR)/png/, pngle.c)
SRC_USERMOD += $(addprefix $(JD9853_MOD_DIR)/png/, miniz.c)
