# LifeTrac v25 — AES-128-GCM golden-vector cross-check (host build).
#
# Compiles lp_crypto_real.cpp's MbedTLS path against the host MbedTLS
# package and runs host_check, which mirrors the Python reference in
# base_station/tests/test_crypto_vectors.py.
#
# Prereqs:
#     Debian/Ubuntu : sudo apt install libmbedtls-dev
#     macOS         : brew install mbedtls
#
# Usage:
#     make crypto-check        # build + run
#     make clean

CXX      ?= g++
CC       ?= gcc
COMMON    = ../../common/lora_proto
CFLAGS   ?= -Wall -Wextra -O2 -I$(COMMON) -DLIFETRAC_USE_REAL_CRYPTO -DLIFETRAC_FORCE_MBEDTLS
CXXFLAGS ?= $(CFLAGS) -std=c++14
LDLIBS   ?= -lmbedcrypto

OBJS = host_check.o lp_crypto_real.o

host_check: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDLIBS)

host_check.o: host_check.c
	$(CC) $(CFLAGS) -c $< -o $@

lp_crypto_real.o: $(COMMON)/lp_crypto_real.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: crypto-check clean
crypto-check: host_check
	./host_check

clean:
	rm -f $(OBJS) host_check
