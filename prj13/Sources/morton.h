#pragma once

// Inverse of Part1By1 - "delete" all odd-indexed bits
inline uint32_t Compact1By1(uint32_t x)
{
	x &= 0x55555555;                  // x = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
	x = (x ^ (x >> 1)) & 0x33333333; // x = --fe --dc --ba --98 --76 --54 --32 --10
	x = (x ^ (x >> 2)) & 0x0f0f0f0f; // x = ---- fedc ---- ba98 ---- 7654 ---- 3210
	x = (x ^ (x >> 4)) & 0x00ff00ff; // x = ---- ---- fedc ba98 ---- ---- 7654 3210
	x = (x ^ (x >> 8)) & 0x0000ffff; // x = ---- ---- ---- ---- fedc ba98 7654 3210
	return x;
}
inline uint32_t DecodeMorton2X(uint32_t code)
{
	return Compact1By1(code >> 0);
}
inline uint32_t DecodeMorton2Y(uint32_t code)
{
	return Compact1By1(code >> 1);
}
inline unsigned int nextPow2(unsigned int v) {
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}
inline unsigned int log2(int in) {
	int i = 0;
	while (in) {
		++i;
		in /= 2;
	}
	return i;
}
inline uint32_t flipLevels(int z, int levels) {
	int newZ = 0;
	int mask;

	for (int i = 0; i < levels; ++i) {
		int mask = 3;
		mask <<= (i * 2);
		int quadrent = (mask & z) >> (i * 2);
		newZ |= (quadrent << ((levels - i - 2)) * 2);
	}
	return newZ;
}
inline int getLevel(int z, int levels) {
	int i;
	for (i = 0; i < levels; ++i) {
		int mask = 3;
		mask <<= (i * 2);
		int quadrant = (mask & z) >> (i * 2);
		if (quadrant != 0) break;
	}
	return levels - i;
}