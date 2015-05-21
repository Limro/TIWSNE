#include "Huffman.h"


module HuffmanC{
	provides interface Compression;
	uses interface Leds;
}
implementation{
	

/*************************************************************************
* _Huffman_InitBitstream() - Initialize a bitstream.
*************************************************************************/

void _Huffman_InitBitstream(huff_bitstream_t *stream, uint8_t *buf)
{
	stream->BytePtr = buf;
	stream->BitPos = 0;
}
	
/*************************************************************************
* _Huffman_WriteBits() - Write bits to a bitstream.
*************************************************************************/

void _Huffman_WriteBits(huff_bitstream_t *stream, uint16_t x, uint16_t bits)
{
	uint16_t  bit, count;
	uint8_t *buf;
	uint16_t  mask;

	/* Get current stream state */
	buf = stream->BytePtr;
	bit = stream->BitPos;

	/* Append bits */
	mask = 1 << (bits - 1);
	for (count = 0; count < bits; ++count)
	{
		*buf = (*buf & (0xff ^ (1 << (7 - bit)))) +
			((x & mask ? 1 : 0) << (7 - bit));
		x <<= 1;
		bit = (bit + 1) & 7;
		if (!bit)
			++buf;
	}

	/* Store new stream state */
	stream->BytePtr = buf;
	stream->BitPos = bit;
}
	
/*************************************************************************
* _Huffman_Hist() - Calculate (sorted) histogram for a block of data.
*************************************************************************/


void _Huffman_Hist(uint8_t *in, huff_sym_t *sym, uint16_t size)

{
	int16_t k;


	/* Clear/init histogram */
	for (k = 0; k < 256; ++k)
	{
		sym[k].Symbol = k;
		sym[k].Count = 0;
		sym[k].Code = 0;
		sym[k].Bits = 0;
	}

	/* Build histogram */
	for (k = size; k; --k)
		sym[*in++].Count++;
}

/*************************************************************************
* _Huffman_StoreTree() - Store a Huffman tree in the output stream and
* in a look-up-table (a symbol array).
*************************************************************************/

void _Huffman_StoreTree(huff_encodenode_t *node, huff_sym_t *sym,
	huff_bitstream_t *stream, uint16_t code, uint16_t bits)
{
	uint16_t sym_idx;

	/* Is this a leaf node? */
	if (node->Symbol >= 0)
	{
		/* Append symbol to tree description */
		_Huffman_WriteBits(stream, 1, 1);
		_Huffman_WriteBits(stream, node->Symbol, 8);

		/* Find symbol index */
		for (sym_idx = 0; sym_idx < 256; ++sym_idx)
		if (sym[sym_idx].Symbol == node->Symbol) break;

		/* Store code info in symbol array */
		sym[sym_idx].Code = code;
		sym[sym_idx].Bits = bits;
		call Leds.led0Toggle();
		return;
	}
	else
		/* This was not a leaf node */
		_Huffman_WriteBits(stream, 0, 1);

	/* Branch A */
	_Huffman_StoreTree(node->ChildA , sym, stream, (code << 1) + 0, bits + 1);

	/* Branch B */
	_Huffman_StoreTree(node->ChildB , sym, stream, (code << 1) + 1, bits + 1);
	
}

/*************************************************************************
* _Huffman_MakeTree() - Generate a Huffman tree.
*************************************************************************/

void _Huffman_MakeTree(huff_sym_t *sym, huff_bitstream_t *stream)
{
	huff_encodenode_t nodes[MAX_TREE_NODES];
	huff_encodenode_t *node_1, *node_2, *root;
	uint16_t k, num_symbols, nodes_left, next_idx;

	/* Initialize all leaf nodes */
	num_symbols = 0;
	for (k = 0; k < 256; ++k)
	if (sym[k].Count > 0)
	{
		nodes[num_symbols].Symbol = sym[k].Symbol;
		nodes[num_symbols].Count = sym[k].Count;
		nodes[num_symbols].ChildA = NULL;
		nodes[num_symbols].ChildB = NULL;
		++num_symbols;
	}

	

	/* Build tree by joining the lightest nodes until there is only
	one node left (the root node). */
	root = NULL;
	nodes_left = num_symbols;
	next_idx = num_symbols;
	while (nodes_left > 1)
	{
		/* Find the two lightest nodes */
		node_1 = NULL;
		node_2 = NULL;
		for (k = 0; k < next_idx; ++k)
		{

			if (nodes[k].Count > 0)
			{
				if (!node_1 || (nodes[k].Count <= node_1->Count))
				{
					node_2 = node_1;
					node_1 = &nodes[k];
				}
				else if (!node_2 || (nodes[k].Count <= node_2->Count))
				{
					node_2 = &nodes[k];
				}
			}
		}

		/* Join the two nodes into a new parent node */

		root = &nodes[next_idx];
		root->ChildA = node_1;
		root->ChildB = node_2;
		root->Count = node_1->Count + node_2->Count;
		root->Symbol = -1;
		node_1->Count = 0;
		node_2->Count = 0;
		++next_idx;
		--nodes_left;
	}


	/* Store the tree in the output stream, and in the sym[] array (the
	latter is used as a look-up-table for faster encoding) */
	if (root)
	{
		_Huffman_StoreTree(root, sym, stream, 0, 0);
	}
	else
	{
		/* Special case: only one symbol => no binary tree */
		root = &nodes[0];
		_Huffman_StoreTree(root, sym, stream, 0, 1);
	}
}


/*************************************************************************
* Huffman_Compress() - Compress a block of data using a Huffman coder.
*  in     - Input (uncompressed) buffer.
*  out    - Output (compressed) buffer. This buffer must be 384 bytes
*           larger than the input buffer.
*  insize - Number of input bytes.
* The function returns the size of the compressed data.
*************************************************************************/

int16_t Huffman_Compress(uint8_t *in, uint8_t *out, uint16_t insize)

{
	huff_sym_t       sym[256], tmp;
	huff_bitstream_t stream;
	uint16_t     k, total_bytes, swaps, symbol;


	
	/* Do we have anything to compress? */
	if (insize < 1) return 0;

	/* Initialize bitstream */
	_Huffman_InitBitstream(&stream, out);
	
	call Leds.led0On();
	
	/* Calculate and sort histogram for input data */
	_Huffman_Hist(in, sym, insize);

	call Leds.led1On();

	/* Build Huffman tree */
	_Huffman_MakeTree(sym, &stream);
	
	call Leds.led2On();
	
	/* Sort histogram - first symbol first (bubble sort) */
	do
	{
		swaps = 0;
		for (k = 0; k < 255; ++k)
		{
			if (sym[k].Symbol > sym[k + 1].Symbol)
			{
				tmp = sym[k];
				sym[k] = sym[k + 1];
				sym[k + 1] = tmp;
				swaps = 1;
			}
		}
	} while (swaps);

	/* Encode input stream */
	for (k = 0; k < insize; ++k)
	{
		symbol = in[k];
		_Huffman_WriteBits(&stream, sym[symbol].Code,
			sym[symbol].Bits);
	}

	/* Calculate size of output data */
	total_bytes = (int16_t)(stream.BytePtr - out);

	if (stream.BitPos > 0)
	{
		++total_bytes;
	}

	return total_bytes;
}

command uint16_t Compression.Compress(uint8_t *in, uint8_t *out, uint16_t insize){
	return Huffman_Compress(in, out, insize);
}


uint16_t _Huffman_ReadBit(huff_bitstream_t *stream)
{
	uint16_t  x, bit;
	uint8_t *buf;

	/* Get current stream state */
	buf = stream->BytePtr;
	bit = stream->BitPos;

	/* Extract bit */
	x = (*buf & (1 << (7 - bit))) ? 1 : 0;
	bit = (bit + 1) & 7;
	if (!bit)
	{
		++buf;
	}

	/* Store new stream state */
	stream->BitPos = bit;
	stream->BytePtr = buf;
	return x;
}


/*************************************************************************
* _Huffman_Read8Bits() - Read eight bits from a bitstream.
*************************************************************************/

uint16_t _Huffman_Read8Bits(huff_bitstream_t *stream)
{
	uint16_t  x, bit;
	uint8_t *buf;


	/* Get current stream state */
	buf = stream->BytePtr;
	bit = stream->BitPos;

	/* Extract byte */
	x = (*buf << bit) | (buf[1] >> (8 - bit));
	++buf;

	/* Store new stream state */
	stream->BytePtr = buf;

	return x;
}

/*************************************************************************
* _Huffman_RecoverTree() - Recover a Huffman tree from a bitstream.
*************************************************************************/

huff_decodenode_t * _Huffman_RecoverTree(huff_decodenode_t *nodes,
	huff_bitstream_t *stream, uint16_t *nodenum)
{
	huff_decodenode_t * this_node;

	/* Pick a node from the node array */
	this_node = &nodes[*nodenum];
	*nodenum = *nodenum + 1;

	/* Clear the node */
	this_node->Symbol = -1;
	this_node->ChildA = NULL;
	this_node->ChildB = NULL;

	/* Is this a leaf node? */
	if (_Huffman_ReadBit(stream))
	{
		/* Get symbol from tree description and store in lead node */
		this_node->Symbol = _Huffman_Read8Bits(stream);
		//    this_node->Symbol = _Huffman_Read16Bits( stream );

		return this_node;
	}

	/* Get branch A */
	this_node->ChildA = _Huffman_RecoverTree(nodes, stream, nodenum);

	/* Get branch B */
	this_node->ChildB = _Huffman_RecoverTree(nodes, stream, nodenum);

	return this_node;
}



void Huffman_Uncompress(uint8_t *in, uint8_t *out,
	uint16_t insize, uint16_t outsize)

{
	huff_decodenode_t nodes[MAX_TREE_NODES], *root, *node;
	huff_bitstream_t  stream;
	uint16_t      k, node_count;
	uint8_t     *buf;


	/* Do we have anything to decompress? */
	if (insize < 1) return;

	/* Initialize bitstream */
	_Huffman_InitBitstream(&stream, in);

	/* Recover Huffman tree */
	node_count = 0;

	root = _Huffman_RecoverTree(nodes, &stream, &node_count);

	/* Decode input stream */
	buf = out;
	for (k = 0; k < outsize; ++k)
	{
		/* Traverse tree until we find a matching leaf node */
		node = root;
		while (node->Symbol < 0)
		{
			/* Get next node */
			if (_Huffman_ReadBit(&stream))
				node = node->ChildB;
			else
				node = node->ChildA;
		}

		/* We found the matching leaf node and have the symbol */
		*buf++ = (uint8_t)node->Symbol;


	}

}

command void Compression.Decompress(uint8_t *in, uint8_t *out, uint16_t insize,  uint16_t outsize){
	Huffman_Uncompress(in, out, insize, outsize);
	return;
}


}