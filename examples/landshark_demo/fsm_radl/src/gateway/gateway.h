/*
 * This file defines generic identity node classes.
 * 
 * The class name is <node_name>_ID<x>
 * with <node_name> the radl node it is compiled for,
 * with <x> the number of inputs.
 *
 * The <y>th input (resp. output) has to be named i<y> (resp. o<y>).
 *
 * Note that being the identity, it forwards the flags instead of blending them.
 *
 */

#include <string.h>

#include RADL_HEADER


class RADL_NODE_PREFIX(_ID1) {
	public:
	void step(const radl_in_t * i, const radl_in_flags_t *inf,
		  radl_out_t * o, radl_out_flags_t * of) {
		memcpy(&o->o1, &i->i1, sizeof(i->i1));
		of->o1 =inf->i1;
	}
};

class RADL_NODE_PREFIX(_ID2) {
	public:
	void step(const radl_in_t * i, const radl_in_flags_t *inf,
		  radl_out_t * o, radl_out_flags_t * of) {
		memcpy(&o->o1, &i->i1, sizeof(i->i1));
		memcpy(&o->o2, &i->i2, sizeof(i->i2));
		of->o1 =inf->i1;
		of->o2 =inf->i2;
	}
};

class RADL_NODE_PREFIX(_ID3) {
	public:
	void step(const radl_in_t * i, const radl_in_flags_t *inf,
		  radl_out_t * o, radl_out_flags_t * of) {
		memcpy(&o->o1, &i->i1, sizeof(i->i1));
		memcpy(&o->o2, &i->i2, sizeof(i->i2));
		memcpy(&o->o3, &i->i3, sizeof(i->i3));
		of->o1 =inf->i1;
		of->o2 =inf->i2;
		of->o3 =inf->i3;
	}
};

class RADL_NODE_PREFIX(_ID4) {
	public:
	void step(const radl_in_t * i, const radl_in_flags_t *inf,
		  radl_out_t * o, radl_out_flags_t * of) {
		memcpy(&o->o1, &i->i1, sizeof(i->i1));
		memcpy(&o->o2, &i->i2, sizeof(i->i2));
		memcpy(&o->o3, &i->i3, sizeof(i->i3));
		memcpy(&o->o4, &i->i4, sizeof(i->i4));
		of->o1 =inf->i1;
		of->o2 =inf->i2;
		of->o3 =inf->i3;
		of->o4 =inf->i4;
	}
};

class RADL_NODE_PREFIX(_ID5) {
	public:
	void step(const radl_in_t * i, const radl_in_flags_t *inf,
		  radl_out_t * o, radl_out_flags_t * of) {
		memcpy(&o->o1, &i->i1, sizeof(i->i1));
		memcpy(&o->o2, &i->i2, sizeof(i->i2));
		memcpy(&o->o3, &i->i3, sizeof(i->i3));
		memcpy(&o->o4, &i->i4, sizeof(i->i4));
		memcpy(&o->o5, &i->i5, sizeof(i->i5));
		of->o1 =inf->i1;
		of->o2 =inf->i2;
		of->o3 =inf->i3;
		of->o4 =inf->i4;
		of->o5 =inf->i5;
	}
};

class RADL_NODE_PREFIX(_ID6) {
	public:
	void step(const radl_in_t * i, const radl_in_flags_t *inf,
		  radl_out_t * o, radl_out_flags_t * of) {
		memcpy(&o->o1, &i->i1, sizeof(i->i1));
		memcpy(&o->o2, &i->i2, sizeof(i->i2));
		memcpy(&o->o3, &i->i3, sizeof(i->i3));
		memcpy(&o->o4, &i->i4, sizeof(i->i4));
		memcpy(&o->o5, &i->i5, sizeof(i->i5));
		memcpy(&o->o6, &i->i6, sizeof(i->i6));
		of->o1 =inf->i1;
		of->o2 =inf->i2;
		of->o3 =inf->i3;
		of->o4 =inf->i4;
		of->o5 =inf->i5;
		of->o6 =inf->i6;
	}
};

class RADL_NODE_PREFIX(_ID7) {
	public:
	void step(const radl_in_t * i, const radl_in_flags_t *inf,
		  radl_out_t * o, radl_out_flags_t * of) {
		memcpy(&o->o1, &i->i1, sizeof(i->i1));
		memcpy(&o->o2, &i->i2, sizeof(i->i2));
		memcpy(&o->o3, &i->i3, sizeof(i->i3));
		memcpy(&o->o4, &i->i4, sizeof(i->i4));
		memcpy(&o->o5, &i->i5, sizeof(i->i5));
		memcpy(&o->o6, &i->i6, sizeof(i->i6));
		memcpy(&o->o7, &i->i7, sizeof(i->i7));
		of->o1 =inf->i1;
		of->o2 =inf->i2;
		of->o3 =inf->i3;
		of->o4 =inf->i4;
		of->o5 =inf->i5;
		of->o6 =inf->i6;
		of->o7 =inf->i7;
	}
};

class RADL_NODE_PREFIX(_ID8) {
	public:
	void step(const radl_in_t * i, const radl_in_flags_t *inf,
		  radl_out_t * o, radl_out_flags_t * of) {
		memcpy(&o->o1, &i->i1, sizeof(i->i1));
		memcpy(&o->o2, &i->i2, sizeof(i->i2));
		memcpy(&o->o3, &i->i3, sizeof(i->i3));
		memcpy(&o->o4, &i->i4, sizeof(i->i4));
		memcpy(&o->o5, &i->i5, sizeof(i->i5));
		memcpy(&o->o6, &i->i6, sizeof(i->i6));
		memcpy(&o->o7, &i->i7, sizeof(i->i7));
		memcpy(&o->o8, &i->i8, sizeof(i->i8));
		of->o1 =inf->i1;
		of->o2 =inf->i2;
		of->o3 =inf->i3;
		of->o4 =inf->i4;
		of->o5 =inf->i5;
		of->o6 =inf->i6;
		of->o7 =inf->i7;
		of->o8 =inf->i8;
	}
};

class RADL_NODE_PREFIX(_ID9) {
	public:
	void step(const radl_in_t * i, const radl_in_flags_t *inf,
		  radl_out_t * o, radl_out_flags_t * of) {
		memcpy(&o->o1, &i->i1, sizeof(i->i1));
		memcpy(&o->o2, &i->i2, sizeof(i->i2));
		memcpy(&o->o3, &i->i3, sizeof(i->i3));
		memcpy(&o->o4, &i->i4, sizeof(i->i4));
		memcpy(&o->o5, &i->i5, sizeof(i->i5));
		memcpy(&o->o6, &i->i6, sizeof(i->i6));
		memcpy(&o->o7, &i->i7, sizeof(i->i7));
		memcpy(&o->o8, &i->i8, sizeof(i->i8));
		memcpy(&o->o9, &i->i9, sizeof(i->i9));
		of->o1 =inf->i1;
		of->o2 =inf->i2;
		of->o3 =inf->i3;
		of->o4 =inf->i4;
		of->o5 =inf->i5;
		of->o6 =inf->i6;
		of->o7 =inf->i7;
		of->o8 =inf->i8;
		of->o9 =inf->i9;
	}
};
