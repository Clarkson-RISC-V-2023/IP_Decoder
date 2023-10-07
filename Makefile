.PHONY: all clean rclean

# ONLY CHANGE THIS
# -------------------------------------------------
FILENAME = decoder
# -------------------------------------------------

# File Constants
RTL = rtl/
VERIF = verif/
DESIGN = $(RTL)$(FILENAME).sv
DESIGN_TB_FILE = tb_$(FILENAME)
DESIGN_TB = $(VERIF)$(DESIGN_TB_FILE).sv
PWD = $(shell pwd)/
VCD = $(DESIGN_TB_FILE).vcd

# Directory Constants
OUT_DIR = ./out/
XSIM_DIR = xsim.dir/

# Executable Constants (VLOG)
VLOG_CC = xvlog
VLOG_CC_OPTIONS = --sv -nolog
VLOG_FILE = xvlog.pb
# Executable Constants (ELAB)
ELAB_CC = xelab
ELAB_CC_OPTIONS = -debug typical
# Executable Constants (SIM)
SIM_CC = xsim
SIM_CC_OPTIONS = -R 

all:

	# Building the xsim.dir, Directory
	rm -rf $(OUT_DIR)
	$(VLOG_CC) $(VLOG_CC_OPTIONS) $(DESIGN) $(DESIGN_TB)


	# Moving the files into a temporary directory
	mkdir -p $(OUT_DIR)
	mv $(XSIM_DIR) $(OUT_DIR)$(XSIM_DIR)
	mv $(VLOG_FILE) $(OUT_DIR)$(VLOG_FILE)

	cd $(OUT_DIR) && $(ELAB_CC) $(ELAB_CC_OPTIONS) $(DESIGN_TB_FILE)

	cd $(OUT_DIR) && $(SIM_CC) $(DESIGN_TB_FILE) $(SIM_CC_OPTIONS) 

clean:
	rm -rf $(OUT_DIR)
	rm -rf $(XSIM_DIR)
	rm -rf *.pb *.log *.jou *.wdb

rclean: clean
	rm -rf *.vcd
