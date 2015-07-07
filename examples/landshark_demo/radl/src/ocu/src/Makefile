PROTO_SRC_DIR=proto

all: proto
	
.PHONY: proto

proto: 
	protoc -I=$(PROTO_SRC_DIR) --cpp_out=. $(PROTO_SRC_DIR)/ocu.proto

