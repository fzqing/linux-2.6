#
# Makefile for misc devices that really don't fit anywhere else.
#
obj- := misc.o	# Dummy rule to force built-in.o to be made

obj-$(CONFIG_IBM_ASM)	+= ibmasm/

obj-$(CONFIG_MCP)              += mcp-core.o
obj-$(CONFIG_MCP_UCB1200)      += ucb1x00-core.o
obj-$(CONFIG_MCP_UCB1200_AUDIO)        += ucb1x00-audio.o
obj-$(CONFIG_MCP_UCB1200_TS)   += ucb1x00-ts.o

ifeq ($(CONFIG_SA1100_ASSABET),y)
obj-$(CONFIG_MCP_UCB1200)      += ucb1x00-assabet.o
endif

obj-$(CONFIG_MCP_SA1100)       += mcp-sa1100.o

ucb1400-core-y := ucb1x00-core.o mcp-ac97.o
obj-$(CONFIG_UCB1400_TS) += ucb1400-core.o ucb1x00-ts.o
