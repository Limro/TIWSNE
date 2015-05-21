
//The maximum number of nodes in the Huffman tree is 2^(8+1)-1 = 511 
#define MAX_TREE_NODES 511

typedef struct {
	uint8_t *BytePtr;
	uint16_t  BitPos;
} huff_bitstream_t;

typedef struct {
	int16_t Symbol;
	//    uint16_t Count;
	uint32_t Count;
	uint16_t Code;
	uint16_t Bits;
} huff_sym_t;


typedef struct huff_encodenode_struct {
	struct huff_encodenode_struct *ChildA;
	struct huff_encodenode_struct *ChildB;
	//int16_t Count;
	int32_t Count;
	int16_t Symbol;
}huff_encodenode_t;


typedef struct huff_decodenode_struct {
	struct huff_decodenode_struct *ChildA;
	struct huff_decodenode_struct *ChildB;
	int16_t Symbol;
}huff_decodenode_t;
