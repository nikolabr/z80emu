#include "z80.h"
#include <gtk/gtk.h>
#include <pthread.h>

uint32_t cycle_count = 0;
pthread_t emulation_thread;
GtkEntry * register_a, *register_f, *register_b, *register_c, *register_d, *register_e, *register_h, *register_l,
		 *register_a_, *register_f_, *register_b_, *register_c_, *register_d_, *register_e_, *register_h_, *register_l_,
		 *register_i, *register_r, *register_ix, *register_iy, *register_pc, *register_sp, 
		 *iff1, *iff2, *im, *last_instruction;

#define UPDATE_UI_ELEMENT_8BIT(widget, src)  		\
	snprintf(tmp, 3, "%.2X", z80_state->src & 0xFF);\
	gtk_entry_set_text(widget, tmp);		\

#define UPDATE_UI_ELEMENT_16BIT(widget, src)  		\
	snprintf(tmp, 5, "%.4X", z80_state->src);   	\
	gtk_entry_set_text(widget, tmp);		\


#define TO_FILL nop

#define INSTRUCTION(prefix, opcode) static inline void prefix##opcode(Z80State * state)

#define XY_INSTRUCTION(prefix, opcode) static inline void prefix##opcode(Z80State * state, uint16_t * ix_reg)
#define XYCB_INSTRUCTION(prefix, opcode) static inline void prefix##opcode(Z80State * state, uint16_t addr)
#define LEA uint16_t addr = xy_ea(state, ix_reg)

#define SZ_FLAGS(src) (src & 0x80) | (!(uint8_t)src << 6)
#define SZ_FLAGS_16(src) ((src & 0x8000) >> 8) | (!src << 6)

#define H_FLAG(x, y, z) ( (x ^ y ^ z) & 16)
#define H_FLAG_16(x, y, z) ( ((x ^ y ^ z) >> 8) & 16)

#define A	  state->a
#define F	  state->f
#define B	  state->b
#define C	  state->c
#define D	  state->d
#define E	  state->e
#define H	  state->h
#define L	  state->l

#define AF 	  state->af
#define BC 	  state->bc
#define DE 	  state->de
#define HL 	  state->hl

#define PC        state->pc
#define SP        state->sp
#define IX        state->ix
#define IY        state->iy

#define F_C 0x01
#define F_N 0x02 
#define F_P 0x04 
#define F_H 0x10 
#define F_Z 0x40 
#define F_S 0x80 


static inline uint8_t check_overflow_add8(int8_t a, int8_t b){
	int16_t t = a + b;

	return t < -128 || t > 127 ? 4 : 0;
}

static inline uint8_t check_overflow_adc8(int8_t a, int8_t b, int8_t carry){
	int16_t t = a + b + carry;

	return t < -128 || t > 127 ? 4 : 0;
}

static inline uint8_t check_overflow_sub8(int8_t a, int8_t b){
	int16_t t = a - b;

	return t < -128 || t > 127 ? 4 : 0;
}

static inline uint8_t check_overflow_sbc8(int8_t a, int8_t b, int8_t carry){
	int16_t t = a - b - carry;

	return t < -128 || t > 127 ? 4 : 0;
}

static inline uint8_t check_overflow_inc8(int8_t val){
	uint8_t tmp = val; 
	return tmp == 127 ? 4 : 0;
}

static inline uint8_t check_overflow_dec8(int8_t val){
	uint8_t tmp = val; 
	return tmp == 128 ? 4 : 0;
}

static inline uint8_t check_overflow_adc16(int16_t a, int16_t b, int8_t carry){
	int32_t t = a + b + carry;

	return t < -32768 || t > 32767 ? 4 : 0;
}

static inline uint8_t check_overflow_sbc16(int16_t a, int16_t b, int8_t carry){
	int32_t t = a - b - carry;

	return t < -32768 || t > 32767 ? 4 : 0;
}

Z80State state;
Z80State * z80_state = &state;
uint8_t z80_ram[65536];

static inline void invalid(){
	//printf("Invalid instruction!\n");
}

typedef enum z80_flag {
	CARRY_FLAG, 
	ADD_SUB_FLAG, // Add/subtract flag
	PV_FLAG, // Parity/overflow flag
	UNUSED_FLAG_1,
	HALF_CARRY_FLAG,
	UNUSED_FLAG_2,
	ZERO_FLAG,
	SIGN_FLAG
} Z80Flag;

static inline uint8_t fetch_byte(Z80State * state){ 
	uint8_t res = z80_ram[state->pc];
	state->pc++;
	return res;
}

static inline uint16_t fetch_word(Z80State * state){ 
	uint16_t res; 
	res = fetch_byte(state);
	res |= fetch_byte(state) << 8;
	return res;
}

static inline uint8_t memory_read(uint16_t addr){
	return z80_ram[addr];
}

static inline uint16_t memory_read_16(uint16_t addr){
	uint16_t tmp;
	tmp = memory_read(addr);
	tmp |= memory_read(addr + 1) << 8;
	return tmp;
}

static inline void memory_write(uint16_t addr, uint8_t val){
	z80_ram[addr] = val;
}

static inline void memory_write_16(uint16_t addr, uint16_t val){
	memory_write(addr, val & 0xFF);
	memory_write(addr + 1, val >> 8);
}

static inline uint16_t xy_ea(Z80State * state, uint16_t * ix_reg){
	uint16_t tmp = *ix_reg + (int8_t)fetch_byte(state); 
	return tmp;
}

const uint8_t parity_table[256] = {
	4, 0, 0, 4, 0, 4, 4, 0, 0, 4, 4, 0, 4, 0, 0, 4,
	0, 4, 4, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, 4, 4, 0,
	0, 4, 4, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, 4, 4, 0,
	4, 0, 0, 4, 0, 4, 4, 0, 0, 4, 4, 0, 4, 0, 0, 4,
	0, 4, 4, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, 4, 4, 0,
	4, 0, 0, 4, 0, 4, 4, 0, 0, 4, 4, 0, 4, 0, 0, 4,
	4, 0, 0, 4, 0, 4, 4, 0, 0, 4, 4, 0, 4, 0, 0, 4,
	0, 4, 4, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, 4, 4, 0,
	0, 4, 4, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, 4, 4, 0,
	4, 0, 0, 4, 0, 4, 4, 0, 0, 4, 4, 0, 4, 0, 0, 4,
	4, 0, 0, 4, 0, 4, 4, 0, 0, 4, 4, 0, 4, 0, 0, 4,
	0, 4, 4, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, 4, 4, 0,
	4, 0, 0, 4, 0, 4, 4, 0, 0, 4, 4, 0, 4, 0, 0, 4,
	0, 4, 4, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, 4, 4, 0,
	0, 4, 4, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, 4, 4, 0,
	4, 0, 0, 4, 0, 4, 4, 0, 0, 4, 4, 0, 4, 0, 0, 4
};

static inline void ld_a_i(Z80State * state){
	A = state->i;
	F = (F & F_C) | (state->iff2 << 2) | SZ_FLAGS(A);
}

static inline void ld_a_r(Z80State * state){
	A = state->r;
	F = (F & F_C) | (state->iff2 << 2) | SZ_FLAGS(A);
}

static inline void push(Z80State * state, uint16_t * src){
	SP -= 2;
	memory_write_16(SP, *src);
}

static inline void pop(Z80State * state, uint16_t * dest){
	*dest = memory_read_16(SP);
	SP += 2;
}

// Exchange group

static inline void ex_de_hl(Z80State * state){
	uint16_t tmp;
	tmp = HL;
	HL = DE;
	DE = tmp;
}

static inline void ex_af_af_(Z80State * state){
	uint16_t tmp;
	tmp = state->af_;
	state->af_ = AF;
	AF = tmp;
}

static inline void exx(Z80State * state){
	uint16_t tmp;

	tmp = state->bc_;
	state->bc_ = BC;
	BC = tmp;

	tmp = state->de_;
	state->de_ = DE;
	DE = tmp;

	tmp = state->hl_;
	state->hl_ = HL;
	HL = tmp;
}

static inline void ex_ptrsp_hl(Z80State * state){
	uint16_t tmp = HL;
	HL = memory_read_16(SP);
	memory_write_16(SP, tmp);
}

static inline void ex_ptrsp_xy(Z80State * state, uint16_t * index_reg){
	uint16_t tmp = memory_read_16(SP);
	memory_write_16(SP, *index_reg);
	*index_reg = tmp;
}

static inline void ldi(Z80State * state){
	memory_write(DE, memory_read(HL));
	DE++;
	HL++;
	BC--;
	
	F = (F & ~(F_N | F_P | F_H)) | ((BC != 0) << 2);
}

static inline void ldir(Z80State * state){
	while (BC != 0) 
		ldi(state);
}

static inline void ldd(Z80State * state){
	memory_write(DE, memory_read(HL));
	DE--;
	HL--;
	BC--;
	
	F = (F & ~(F_N | F_P | F_H)) | ((BC != 0) << 2);
}

static inline void lddr(Z80State * state){
	ldd(state);

	if (BC != 0) 
		PC -= 2;
	else {} // Different cycle count 

}

static inline void cpi(Z80State * state){				
	int16_t x, y;
	x = memory_read(HL);
	y = A - x;

	HL++;
	BC--;

	F = (F & F_C) | H_FLAG(x, y, A) | F_N | ((BC != 0) << 2) | SZ_FLAGS(y);		
}

static inline void cpir(Z80State * state){				
	cpi(state);

	if (BC != 0 && !(F & F_Z)) 
		PC -= 2;

}

static inline void cpd(Z80State * state){				
	int16_t x, y;
	x = memory_read(HL);
	y = A - x;

	HL--;
	BC--;

	F = (F & F_C) | H_FLAG(x, y, A) | F_N | ((BC != 0) << 2) | SZ_FLAGS(y);		
}

static inline void cpdr(Z80State * state){				
	cpd(state);

	if (BC != 0 && !(F & F_Z)) 
		PC -= 2;

}

// 8-bit arithemetic

static inline void add_8(Z80State * state, uint8_t * src){ 
	int16_t x, y; 
	x = *src; 
	y = A + x;

	F = H_FLAG(A, x, y) | ( (uint16_t) y > 255) | check_overflow_add8(A, x) | SZ_FLAGS(y);

	A = y;
}

static inline void adc_8(Z80State * state, uint8_t * src){ 
	int16_t x, y;
	uint8_t t = F & F_C;
	x = *src;
	y = A + x + t;
	
	F = H_FLAG(A, x, y) | ( (uint16_t) y > 255) | check_overflow_adc8(A, x, t) | SZ_FLAGS(y);
	
	A = y;
}

static inline void sub_8(Z80State * state, uint8_t * src){
	int16_t x, y; 
	x = *src;
	y = A - x;

	F = H_FLAG(A, x, y) | (A < x) | check_overflow_sub8(A, x) | F_N | SZ_FLAGS(y);

	A = y;
}

static inline void sbc_8(Z80State * state, uint8_t * src){
	int16_t x, y; 
	uint8_t t = F & 1;
	x = *src;
	y = A - x - t;

	F = H_FLAG(A, x, y) | (y < 0) | check_overflow_sbc8(A, x, t) | F_N | SZ_FLAGS(y);

	A = y;
}

static inline void and_8(Z80State * state, uint8_t * src){ 
	int16_t x, y;
	x = *src;
	y = A & x; 

	F = F_H | parity_table[y] | SZ_FLAGS(y);

	A = y;
}

static inline void or_8(Z80State * state, uint8_t * src){ 
	int16_t x, y;
	x = *src;
	y = A | x; 

	F = parity_table[y] | SZ_FLAGS(y);

	A = y;
}

static inline void xor_8(Z80State * state, uint8_t * src){ 
	int16_t x, y;
	x = *src;
	y = A ^ x; 

	F = parity_table[y] | SZ_FLAGS(y);

	A = y;
}

static inline void cp_8(Z80State * state, uint8_t * src){ 
	int16_t x, y; 
	x = *src; 
	y = A - x; 

	F = H_FLAG(A, x, y) | (A < x) | check_overflow_sub8(A, x) | F_N | SZ_FLAGS(y);
}

							

static inline void inc_r(Z80State * state, uint8_t * src){ 
	int16_t x = *src;
	(*src)++;

	F = (F & 1) | H_FLAG(x, 1, *src) | check_overflow_inc8(x) |  SZ_FLAGS(*src);
}

static inline void dec_r(Z80State * state, uint8_t * src){
	int16_t x = *src;
	(*src)--;

	F = (F & 1) | F_N | H_FLAG(x, 1, *src) | check_overflow_dec8(x) |  SZ_FLAGS(*src);
}

// General-purpose arithmetic and CPU control groups

static inline void daa(Z80State * state){
	uint8_t x = ( (F & F_H) || (A & 0xF) > 9 ) ? 6 : 0;
	x |= ( (F & F_C) || A > 0x99) ? 0x60 : 0;

	uint8_t c = F & F_C;

	uint8_t a = A;
	A += ( F & F_N ) ? -x : x;
	
	F = (F & F_N) | H_FLAG(a, 0, A) | ( c || (a > 0x99) ) | parity_table[A] | SZ_FLAGS(A);
}

static inline void cpl(Z80State * state){
	A = ~(A);

	F |= F_H | F_N; // Set H and N
}

static inline void neg(Z80State * state){
	uint8_t x = A;
	A = 0 - A;

	F = !!x | F_N | ( (x == 0x80) << 2) | H_FLAG(x, 0, A) | SZ_FLAGS(A);
}

static inline void ccf(Z80State * state){
	uint8_t c = F & 1;
	F &= 0xEC;
	F |= c << 4;
	F |= ~c & 1;
}

static inline void scf(Z80State * state){
	F |= 1;
	F &= 0xED;
}

static inline void nop(){
}

// 16-bit arithmetic

static inline void add_hl_ss(Z80State * state, uint16_t * src){
	int32_t x = *src;
	int32_t y = HL + x;

	F = (F & ~(F_H | F_N | F_C)) | H_FLAG_16(x, HL, y) | ( (uint32_t) y > 65535 );
	HL = y;
}

static inline void add_xy_rr(Z80State * state, uint16_t * ix_reg, uint16_t * src){
	int32_t x = *src;
	int32_t y = *ix_reg + x;

	F = (F & ~(F_H | F_N | F_C)) | H_FLAG_16(x, *ix_reg, y) | ( (uint32_t) y > 65535 );
	*ix_reg = y;
}

static inline void adc_hl_ss(Z80State * state, uint16_t * src){
	int32_t x = *src;
	uint8_t t = F & F_C;
	int32_t y = HL + x + t;

	F = check_overflow_adc16(HL, x, t) | H_FLAG_16(x, HL, y) | ( (uint32_t) y > 65535 ) | SZ_FLAGS_16(y);
	HL = y;
}

static inline void sbc_hl_ss(Z80State * state, uint16_t * src){
	int32_t x = *src;
	uint8_t t = F & F_C;
	int32_t y = HL - x - t;

	F = F_N | check_overflow_sbc16(HL, x, t) | H_FLAG_16(x, HL, y) | ( (uint32_t) y > 65535 ) | SZ_FLAGS_16(y);
	HL = y;
}

// Rotate and shift group 

static inline void rlca(Z80State * state){
	A = (A << 1) | (A >> 7);
	F = (F & ~(F_H | F_N | F_C)) | (A & 1);
}

static inline void rla(Z80State * state){
	uint8_t rbit = A >> 7;
	A <<= 1;
	A |= F & 1;
	F = (F & ~(F_H | F_N | F_C)) | rbit;
}

static inline void rrca(Z80State * state){
	F = (F & ~(F_H | F_N | F_C)) | (A & 1);
	A = (A >> 1) | (A << 7);
}

static inline void rra(Z80State * state){
	uint8_t rbit = A & 1;
	A >>= 1;
	A |= (F & F_C) << 7;
	F = (F & ~(F_H | F_N | F_C)) | rbit;
}

static inline void rlc(Z80State * state, uint8_t * src){ 
	*src = (*src << 1) | (*src >> 7);
	F = (*src & F_C) | parity_table[*src] | SZ_FLAGS(*src);
}

static inline void rl(Z80State * state, uint8_t * src){
	uint8_t rbit = *src >> 7;
	*src <<= 1;
	*src |= F & F_C;
	F = rbit | parity_table[*src] | SZ_FLAGS(*src);
}

static inline void rrc(Z80State * state, uint8_t * src){ 
	F = *src & F_C; 
	*src = (*src >> 1) | (*src << 7);

	F |= parity_table[*src] | SZ_FLAGS(*src);
}

static inline void rr(Z80State * state, uint8_t * src){
	uint8_t rbit = *src & 1;
	*src >>= 1;
	*src |= (F & F_C) << 7;
	F = rbit | parity_table[*src] | SZ_FLAGS(*src);
}

static inline void sla(Z80State * state, uint8_t * src){
	F = *src >> 7;
	*src <<= 1;
	F |= parity_table[*src] | SZ_FLAGS(*src);
}

static inline void sll(Z80State * state, uint8_t * src){
	F = *src >> 7;
	*src <<= 1;
	*src |= 1; 
	
	F |= parity_table[*src] | SZ_FLAGS(*src);
}

static inline void sra(Z80State * state, uint8_t * src){ 
	uint8_t tmp = *src & 0x80;
	F = *src & F_C; 
	*src >>= 1;
	*src |= tmp;
	F |= parity_table[*src] | SZ_FLAGS(*src);
}

static inline void srl(Z80State * state, uint8_t * src){ 
	F = *src & 1; 
	*src >>= 1;

	F |= parity_table[*src] | SZ_FLAGS(*src);
}

static inline void rld(Z80State * state){
	uint8_t low_acc = A & 0x0F;
	uint8_t * mem = z80_ram + z80_state->hl;

	A &= 0xF0; A |= *mem >> 4;
	*mem <<= 4; *mem |= low_acc;

	F = (F & F_C) | parity_table[A] | SZ_FLAGS(A);
}

static inline void rrd(Z80State * state){
	uint8_t * mem = z80_ram + z80_state->hl;
	uint8_t low_mem = *mem & 0x0F;

	*mem >>= 4; *mem |= A << 4;
	A &= 0xF0; A |= low_mem;

	F = (F & F_C) | parity_table[A] | SZ_FLAGS(A);
}

static inline void bit(Z80State * state, uint8_t bit, uint8_t *src){
	uint8_t b = (*src >> bit) & 1;

	F = (F & F_C) | F_H | (!b << 6);
}

static inline void set(Z80State * state, uint8_t bit, uint8_t * src){
	*src |= 1 << bit;
}

static inline void res(Z80State * state, uint8_t bit, uint8_t * src){
	*src &= ~(1 << bit);
}

static inline void jp(Z80State * state){
	uint16_t tmp = fetch_word(state);
	PC = tmp;
}

static inline void jp_cond(Z80State * state, uint8_t cond){ 
	if (cond)
		jp(state);
	else
		PC += 2;
}

static inline void jr_cond(Z80State * state, uint8_t cond){
	if (cond){
		int8_t tmp = (int8_t)fetch_byte(state);
		PC += tmp - 1;
	}
	else	
		PC++;
}

static inline void djnz(Z80State * state){
	B--; 
	uint8_t tmp = B == 0 ? 1 : fetch_byte(state) + 1; 
	PC += tmp;
}

static inline void call(Z80State * state){
	PC += 2;
	push(state, &PC);
	uint16_t tmp = memory_read_16(PC - 2);
	PC = tmp;
}

static inline void call_cond(Z80State * state, uint8_t cond){
	if (cond)
		call(state);
	else
		PC += 2;
}

static inline void ret(Z80State * state){
	pop(state, &PC);
}

static inline void ret_cond(Z80State * state, uint8_t cond){
	if (cond)
		ret(state);
}

static inline void rst(Z80State * state, uint8_t p){
	push(state, &PC);
	PC = p;
}

typedef void (*Branch)(Z80State * state);
INSTRUCTION(op, XX) {  };

/* ----------------------------------------- */

INSTRUCTION(ed, 40) { TO_FILL(); } // TODO
INSTRUCTION(ed, 41) { TO_FILL(); } // TODO
INSTRUCTION(ed, 42) { sbc_hl_ss(state, &BC); };
INSTRUCTION(ed, 43) { uint16_t tmp = fetch_word(state); memory_write_16(tmp, BC); };
INSTRUCTION(ed, 44) { neg(state); };
INSTRUCTION(ed, 45) { TO_FILL(); } // TODO
INSTRUCTION(ed, 46) { state->im = 0; };
INSTRUCTION(ed, 47) { state->i = A; };

INSTRUCTION(ed, 48) { TO_FILL(); } // TODO
INSTRUCTION(ed, 49) { TO_FILL(); } // TODO
INSTRUCTION(ed, 4A) { adc_hl_ss(state, &BC); };
INSTRUCTION(ed, 4B) { uint16_t tmp = fetch_word(state); BC = memory_read_16(tmp); };
INSTRUCTION(ed, 4C) { neg(state); };
INSTRUCTION(ed, 4D) { TO_FILL(); } // TODO
INSTRUCTION(ed, 4E) { state->im = 0; };
INSTRUCTION(ed, 4F) { state->r = A; };

INSTRUCTION(ed, 50) { TO_FILL(); } // TODO
INSTRUCTION(ed, 51) { TO_FILL(); } // TODO
INSTRUCTION(ed, 52) { sbc_hl_ss(state, &DE); };
INSTRUCTION(ed, 53) { uint16_t tmp = fetch_word(state); memory_write_16(tmp, DE); };
INSTRUCTION(ed, 54) { neg(state); };
INSTRUCTION(ed, 55) { TO_FILL(); } // TODO
INSTRUCTION(ed, 56) { state->im = 1; };
INSTRUCTION(ed, 57) { ld_a_i(state); };

INSTRUCTION(ed, 58) { TO_FILL(); } // TODO
INSTRUCTION(ed, 59) { TO_FILL(); } // TODO
INSTRUCTION(ed, 5A) { adc_hl_ss(state, &DE); };
INSTRUCTION(ed, 5B) { uint16_t tmp = fetch_word(state); DE = memory_read_16(tmp); };
INSTRUCTION(ed, 5C) { neg(state); };
INSTRUCTION(ed, 5D) { TO_FILL(); } // TODO
INSTRUCTION(ed, 5E) { state->im = 2; };
INSTRUCTION(ed, 5F) { ld_a_r(state); };

INSTRUCTION(ed, 60) { TO_FILL(); } // TODO
INSTRUCTION(ed, 61) { TO_FILL(); } // TODO
INSTRUCTION(ed, 62) { sbc_hl_ss(state, &HL); };
INSTRUCTION(ed, 63) { uint16_t tmp = fetch_word(state); memory_write_16(tmp, HL); };
INSTRUCTION(ed, 64) { neg(state); };
INSTRUCTION(ed, 65) { TO_FILL(); } // TODO
INSTRUCTION(ed, 66) { state->im = 0; };
INSTRUCTION(ed, 67) { rrd(state); };

INSTRUCTION(ed, 68) { TO_FILL(); } // TODO
INSTRUCTION(ed, 69) { TO_FILL(); } // TODO
INSTRUCTION(ed, 6A) { adc_hl_ss(state, &HL); };
INSTRUCTION(ed, 6B) { uint16_t tmp = fetch_word(state); HL = memory_read_16(tmp); };
INSTRUCTION(ed, 6C) { neg(state); };
INSTRUCTION(ed, 6D) { TO_FILL(); } // TODO
INSTRUCTION(ed, 6E) { state->im = 0; };
INSTRUCTION(ed, 6F) { rld(state); };

INSTRUCTION(ed, 70) { TO_FILL(); } // TODO
INSTRUCTION(ed, 71) { TO_FILL(); } // TODO
INSTRUCTION(ed, 72) { sbc_hl_ss(state, &SP); };
INSTRUCTION(ed, 73) { uint16_t tmp = fetch_word(state); memory_write_16(tmp, SP); };
INSTRUCTION(ed, 74) { neg(state); };
INSTRUCTION(ed, 75) { TO_FILL(); } // TODO
INSTRUCTION(ed, 76) { state->im = 1; };
INSTRUCTION(ed, 77) { };

INSTRUCTION(ed, 78) { TO_FILL(); } // TODO
INSTRUCTION(ed, 79) { TO_FILL(); } // TODO
INSTRUCTION(ed, 7A) { adc_hl_ss(state, &SP); };
INSTRUCTION(ed, 7B) { uint16_t tmp = fetch_word(state); SP = memory_read_16(tmp); };
INSTRUCTION(ed, 7C) { neg(state); };
INSTRUCTION(ed, 7D) { TO_FILL(); } // TODO
INSTRUCTION(ed, 7E) { state->im = 2; };
INSTRUCTION(ed, 7F) {  };

INSTRUCTION(ed, A0) { ldi(state); };
INSTRUCTION(ed, A1) { cpi(state); };
INSTRUCTION(ed, A2) { TO_FILL(); }; // TODO
INSTRUCTION(ed, A3) { TO_FILL(); }; // TODO
INSTRUCTION(ed, A8) { ldd(state); };
INSTRUCTION(ed, A9) { cpd(state); };
INSTRUCTION(ed, AA) { TO_FILL(); }; // TODO
INSTRUCTION(ed, AB) { TO_FILL(); }; // TODO

INSTRUCTION(ed, B0) { ldir(state); };
INSTRUCTION(ed, B1) { cpir(state); };
INSTRUCTION(ed, B2) { TO_FILL(); }; // TODO
INSTRUCTION(ed, B3) { TO_FILL(); }; // TODO
INSTRUCTION(ed, B8) { lddr(state); };
INSTRUCTION(ed, B9) { cpdr(state); };
INSTRUCTION(ed, BA) { TO_FILL(); }; // TODO
INSTRUCTION(ed, BB) { TO_FILL(); }; // TODO

/* ----------------------------------------- */

const Branch ed_table[128] = {
	ed40, ed41, ed42, ed43, ed44, ed45, ed46, ed47, ed48, ed49, ed4A, ed4B, ed4C, ed4D, ed4E, ed4F,
	ed50, ed51, ed52, ed53, ed54, ed55, ed56, ed57, ed58, ed59, ed5A, ed5B, ed5C, ed5D, ed5E, ed5F,
	ed60, ed61, ed62, ed63, ed64, ed65, ed66, ed67, ed68, ed69, ed6A, ed6B, ed6C, ed6D, ed6E, ed6F,
	ed70, ed71, ed72, ed73, ed74, ed75, ed76, ed77, ed78, ed79, ed7A, ed7B, ed7C, ed7D, ed7E, ed7F,
	opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX,
	opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX, opXX,
	edA0, edA1, edA2, edA3, opXX, opXX, opXX, opXX, edA8, edA9, edAA, edAB, opXX, opXX, opXX, opXX,
	edB0, edB1, edB2, edB3, opXX, opXX, opXX, opXX, edB8, edB9, edBA, edBB, opXX, opXX, opXX, opXX
};

void ed_sw(Z80State * state){
	ed_table[fetch_byte(state) - 0x40](state);
};

/* ----------------------------------------- */

INSTRUCTION(cb, 00) { rlc(state, &B); } ;
INSTRUCTION(cb, 01) { rlc(state, &C); } ;
INSTRUCTION(cb, 02) { rlc(state, &D); } ;
INSTRUCTION(cb, 03) { rlc(state, &E); } ;
INSTRUCTION(cb, 04) { rlc(state, &H); } ;
INSTRUCTION(cb, 05) { rlc(state, &L); } ;
INSTRUCTION(cb, 06) { uint8_t tmp = memory_read(HL); rlc(state, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 07) { rlc(state, &A); } ;

INSTRUCTION(cb, 08) { rrc(state, &B); } ;
INSTRUCTION(cb, 09) { rrc(state, &C); } ;
INSTRUCTION(cb, 0A) { rrc(state, &D); } ;
INSTRUCTION(cb, 0B) { rrc(state, &E); } ;
INSTRUCTION(cb, 0C) { rrc(state, &H); } ;
INSTRUCTION(cb, 0D) { rrc(state, &L); } ;
INSTRUCTION(cb, 0E) { uint8_t tmp = memory_read(HL); rrc(state, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 0F) { rrc(state, &A); } ;

INSTRUCTION(cb, 10) { rl(state, &B); } ;
INSTRUCTION(cb, 11) { rl(state, &C); } ;
INSTRUCTION(cb, 12) { rl(state, &D); } ;
INSTRUCTION(cb, 13) { rl(state, &E); } ;
INSTRUCTION(cb, 14) { rl(state, &H); } ;
INSTRUCTION(cb, 15) { rl(state, &L); } ;
INSTRUCTION(cb, 16) { uint8_t tmp = memory_read(HL); rl(state, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 17) { rl(state, &A); } ;

INSTRUCTION(cb, 18) { rr(state, &B); } ;
INSTRUCTION(cb, 19) { rr(state, &C); } ;
INSTRUCTION(cb, 1A) { rr(state, &D); } ;
INSTRUCTION(cb, 1B) { rr(state, &E); } ;
INSTRUCTION(cb, 1C) { rr(state, &H); } ;
INSTRUCTION(cb, 1D) { rr(state, &L); } ;
INSTRUCTION(cb, 1E) { uint8_t tmp = memory_read(HL); rr(state, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 1F) { rr(state, &A); } ;

INSTRUCTION(cb, 20) { sla(state, &B); } ;
INSTRUCTION(cb, 21) { sla(state, &C); } ;
INSTRUCTION(cb, 22) { sla(state, &D); } ;
INSTRUCTION(cb, 23) { sla(state, &E); } ;
INSTRUCTION(cb, 24) { sla(state, &H); } ;
INSTRUCTION(cb, 25) { sla(state, &L); } ;
INSTRUCTION(cb, 26) { uint8_t tmp = memory_read(HL); sla(state, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 27) { sla(state, &A); } ;

INSTRUCTION(cb, 28) { sra(state, &B); } ;
INSTRUCTION(cb, 29) { sra(state, &C); } ;
INSTRUCTION(cb, 2A) { sra(state, &D); } ;
INSTRUCTION(cb, 2B) { sra(state, &E); } ;
INSTRUCTION(cb, 2C) { sra(state, &H); } ;
INSTRUCTION(cb, 2D) { sra(state, &L); } ;
INSTRUCTION(cb, 2E) { uint8_t tmp = memory_read(HL); sra(state, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 2F) { sra(state, &A); } ;

INSTRUCTION(cb, 30) { sll(state, &B); } ;
INSTRUCTION(cb, 31) { sll(state, &C); } ;
INSTRUCTION(cb, 32) { sll(state, &D); } ;
INSTRUCTION(cb, 33) { sll(state, &E); } ;
INSTRUCTION(cb, 34) { sll(state, &H); } ;
INSTRUCTION(cb, 35) { sll(state, &L); } ;
INSTRUCTION(cb, 36) { uint8_t tmp = memory_read(HL); sll(state, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 37) { sll(state, &A); } ;

INSTRUCTION(cb, 38) { srl(state, &B); } ;
INSTRUCTION(cb, 39) { srl(state, &C); } ;
INSTRUCTION(cb, 3A) { srl(state, &D); } ;
INSTRUCTION(cb, 3B) { srl(state, &E); } ;
INSTRUCTION(cb, 3C) { srl(state, &H); } ;
INSTRUCTION(cb, 3D) { srl(state, &L); } ;
INSTRUCTION(cb, 3E) { uint8_t tmp = memory_read(HL); srl(state, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 3F) { srl(state, &A); } ;

INSTRUCTION(cb, 40) { bit(state, 0, &B); };
INSTRUCTION(cb, 41) { bit(state, 0, &C); };
INSTRUCTION(cb, 42) { bit(state, 0, &D); };
INSTRUCTION(cb, 43) { bit(state, 0, &E); };
INSTRUCTION(cb, 44) { bit(state, 0, &H); };
INSTRUCTION(cb, 45) { bit(state, 0, &L); };
INSTRUCTION(cb, 46) { uint8_t tmp = memory_read(HL); bit(state, 0, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 47) { bit(state, 0, &A); };

INSTRUCTION(cb, 48) { bit(state, 1, &B); };
INSTRUCTION(cb, 49) { bit(state, 1, &C); };
INSTRUCTION(cb, 4A) { bit(state, 1, &D); };
INSTRUCTION(cb, 4B) { bit(state, 1, &E); };
INSTRUCTION(cb, 4C) { bit(state, 1, &H); };
INSTRUCTION(cb, 4D) { bit(state, 1, &L); };
INSTRUCTION(cb, 4E) { uint8_t tmp = memory_read(HL); bit(state, 1, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 4F) { bit(state, 1, &A); };

INSTRUCTION(cb, 50) { bit(state, 2, &B); };
INSTRUCTION(cb, 51) { bit(state, 2, &C); };
INSTRUCTION(cb, 52) { bit(state, 2, &D); };
INSTRUCTION(cb, 53) { bit(state, 2, &E); };
INSTRUCTION(cb, 54) { bit(state, 2, &H); };
INSTRUCTION(cb, 55) { bit(state, 2, &L); };
INSTRUCTION(cb, 56) { uint8_t tmp = memory_read(HL); bit(state, 2, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 57) { bit(state, 2, &A); };

INSTRUCTION(cb, 58) { bit(state, 3, &B); };
INSTRUCTION(cb, 59) { bit(state, 3, &C); };
INSTRUCTION(cb, 5A) { bit(state, 3, &D); };
INSTRUCTION(cb, 5B) { bit(state, 3, &E); };
INSTRUCTION(cb, 5C) { bit(state, 3, &H); };
INSTRUCTION(cb, 5D) { bit(state, 3, &L); };
INSTRUCTION(cb, 5E) { uint8_t tmp = memory_read(HL); bit(state, 3, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 5F) { bit(state, 3, &A); };

INSTRUCTION(cb, 60) { bit(state, 4, &B); };
INSTRUCTION(cb, 61) { bit(state, 4, &C); };
INSTRUCTION(cb, 62) { bit(state, 4, &D); };
INSTRUCTION(cb, 63) { bit(state, 4, &E); };
INSTRUCTION(cb, 64) { bit(state, 4, &H); };
INSTRUCTION(cb, 65) { bit(state, 4, &L); };
INSTRUCTION(cb, 66) { uint8_t tmp = memory_read(HL); bit(state, 4, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 67) { bit(state, 4, &A); };

INSTRUCTION(cb, 68) { bit(state, 5, &B); };
INSTRUCTION(cb, 69) { bit(state, 5, &C); };
INSTRUCTION(cb, 6A) { bit(state, 5, &D); };
INSTRUCTION(cb, 6B) { bit(state, 5, &E); };
INSTRUCTION(cb, 6C) { bit(state, 5, &H); };
INSTRUCTION(cb, 6D) { bit(state, 5, &L); };
INSTRUCTION(cb, 6E) { uint8_t tmp = memory_read(HL); bit(state, 5, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 6F) { bit(state, 5, &A); };

INSTRUCTION(cb, 70) { bit(state, 6, &B); };
INSTRUCTION(cb, 71) { bit(state, 6, &C); };
INSTRUCTION(cb, 72) { bit(state, 6, &D); };
INSTRUCTION(cb, 73) { bit(state, 6, &E); };
INSTRUCTION(cb, 74) { bit(state, 6, &H); };
INSTRUCTION(cb, 75) { bit(state, 6, &L); };
INSTRUCTION(cb, 76) { uint8_t tmp = memory_read(HL); bit(state, 6, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 77) { bit(state, 6, &A); };

INSTRUCTION(cb, 78) { bit(state, 7, &B); };
INSTRUCTION(cb, 79) { bit(state, 7, &C); };
INSTRUCTION(cb, 7A) { bit(state, 7, &D); };
INSTRUCTION(cb, 7B) { bit(state, 7, &E); };
INSTRUCTION(cb, 7C) { bit(state, 7, &H); };
INSTRUCTION(cb, 7D) { bit(state, 7, &L); };
INSTRUCTION(cb, 7E) { uint8_t tmp = memory_read(HL); bit(state, 7, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 7F) { bit(state, 7, &A); };

INSTRUCTION(cb, 80) { res(state, 0, &B); };
INSTRUCTION(cb, 81) { res(state, 0, &C); };
INSTRUCTION(cb, 82) { res(state, 0, &D); };
INSTRUCTION(cb, 83) { res(state, 0, &E); };
INSTRUCTION(cb, 84) { res(state, 0, &H); };
INSTRUCTION(cb, 85) { res(state, 0, &L); };
INSTRUCTION(cb, 86) { uint8_t tmp = memory_read(HL); res(state, 0, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 87) { res(state, 0, &A); };

INSTRUCTION(cb, 88) { res(state, 1, &B); };
INSTRUCTION(cb, 89) { res(state, 1, &C); };
INSTRUCTION(cb, 8A) { res(state, 1, &D); };
INSTRUCTION(cb, 8B) { res(state, 1, &E); };
INSTRUCTION(cb, 8C) { res(state, 1, &H); };
INSTRUCTION(cb, 8D) { res(state, 1, &L); };
INSTRUCTION(cb, 8E) { uint8_t tmp = memory_read(HL); res(state, 1, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 8F) { res(state, 1, &A); };

INSTRUCTION(cb, 90) { res(state, 2, &B); };
INSTRUCTION(cb, 91) { res(state, 2, &C); };
INSTRUCTION(cb, 92) { res(state, 2, &D); };
INSTRUCTION(cb, 93) { res(state, 2, &E); };
INSTRUCTION(cb, 94) { res(state, 2, &H); };
INSTRUCTION(cb, 95) { res(state, 2, &L); };
INSTRUCTION(cb, 96) { uint8_t tmp = memory_read(HL); res(state, 2, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 97) { res(state, 2, &A); };

INSTRUCTION(cb, 98) { res(state, 3, &B); };
INSTRUCTION(cb, 99) { res(state, 3, &C); };
INSTRUCTION(cb, 9A) { res(state, 3, &D); };
INSTRUCTION(cb, 9B) { res(state, 3, &E); };
INSTRUCTION(cb, 9C) { res(state, 3, &H); };
INSTRUCTION(cb, 9D) { res(state, 3, &L); };
INSTRUCTION(cb, 9E) { uint8_t tmp = memory_read(HL); res(state, 3, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, 9F) { res(state, 3, &A); };

INSTRUCTION(cb, A0) { res(state, 4, &B); };
INSTRUCTION(cb, A1) { res(state, 4, &C); };
INSTRUCTION(cb, A2) { res(state, 4, &D); };
INSTRUCTION(cb, A3) { res(state, 4, &E); };
INSTRUCTION(cb, A4) { res(state, 4, &H); };
INSTRUCTION(cb, A5) { res(state, 4, &L); };
INSTRUCTION(cb, A6) { uint8_t tmp = memory_read(HL); res(state, 4, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, A7) { res(state, 4, &A); };

INSTRUCTION(cb, A8) { res(state, 5, &B); };
INSTRUCTION(cb, A9) { res(state, 5, &C); };
INSTRUCTION(cb, AA) { res(state, 5, &D); };
INSTRUCTION(cb, AB) { res(state, 5, &E); };
INSTRUCTION(cb, AC) { res(state, 5, &H); };
INSTRUCTION(cb, AD) { res(state, 5, &L); };
INSTRUCTION(cb, AE) { uint8_t tmp = memory_read(HL); res(state, 5, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, AF) { res(state, 5, &A); };

INSTRUCTION(cb, B0) { res(state, 6, &B); };
INSTRUCTION(cb, B1) { res(state, 6, &C); };
INSTRUCTION(cb, B2) { res(state, 6, &D); };
INSTRUCTION(cb, B3) { res(state, 6, &E); };
INSTRUCTION(cb, B4) { res(state, 6, &H); };
INSTRUCTION(cb, B5) { res(state, 6, &L); };
INSTRUCTION(cb, B6) { uint8_t tmp = memory_read(HL); res(state, 6, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, B7) { res(state, 6, &A); };

INSTRUCTION(cb, B8) { res(state, 7, &B); };
INSTRUCTION(cb, B9) { res(state, 7, &C); };
INSTRUCTION(cb, BA) { res(state, 7, &D); };
INSTRUCTION(cb, BB) { res(state, 7, &E); };
INSTRUCTION(cb, BC) { res(state, 7, &H); };
INSTRUCTION(cb, BD) { res(state, 7, &L); };
INSTRUCTION(cb, BE) { uint8_t tmp = memory_read(HL); res(state, 7, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, BF) { res(state, 7, &A); };

INSTRUCTION(cb, C0) { set(state, 0, &B); };
INSTRUCTION(cb, C1) { set(state, 0, &C); };
INSTRUCTION(cb, C2) { set(state, 0, &D); };
INSTRUCTION(cb, C3) { set(state, 0, &E); };
INSTRUCTION(cb, C4) { set(state, 0, &H); };
INSTRUCTION(cb, C5) { set(state, 0, &L); };
INSTRUCTION(cb, C6) { uint8_t tmp = memory_read(HL); set(state, 0, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, C7) { set(state, 0, &A); };

INSTRUCTION(cb, C8) { set(state, 1, &B); };
INSTRUCTION(cb, C9) { set(state, 1, &C); };
INSTRUCTION(cb, CA) { set(state, 1, &D); };
INSTRUCTION(cb, CB) { set(state, 1, &E); };
INSTRUCTION(cb, CC) { set(state, 1, &H); };
INSTRUCTION(cb, CD) { set(state, 1, &L); };
INSTRUCTION(cb, CE) { uint8_t tmp = memory_read(HL); set(state, 1, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, CF) { set(state, 1, &A); };

INSTRUCTION(cb, D0) { set(state, 2, &B); };
INSTRUCTION(cb, D1) { set(state, 2, &C); };
INSTRUCTION(cb, D2) { set(state, 2, &D); };
INSTRUCTION(cb, D3) { set(state, 2, &E); };
INSTRUCTION(cb, D4) { set(state, 2, &H); };
INSTRUCTION(cb, D5) { set(state, 2, &L); };
INSTRUCTION(cb, D6) { uint8_t tmp = memory_read(HL); set(state, 2, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, D7) { set(state, 2, &A); };

INSTRUCTION(cb, D8) { set(state, 3, &B); };
INSTRUCTION(cb, D9) { set(state, 3, &C); };
INSTRUCTION(cb, DA) { set(state, 3, &D); };
INSTRUCTION(cb, DB) { set(state, 3, &E); };
INSTRUCTION(cb, DC) { set(state, 3, &H); };
INSTRUCTION(cb, DD) { set(state, 3, &L); };
INSTRUCTION(cb, DE) { uint8_t tmp = memory_read(HL); set(state, 3, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, DF) { set(state, 3, &A); };

INSTRUCTION(cb, E0) { set(state, 4, &B); };
INSTRUCTION(cb, E1) { set(state, 4, &C); };
INSTRUCTION(cb, E2) { set(state, 4, &D); };
INSTRUCTION(cb, E3) { set(state, 4, &E); };
INSTRUCTION(cb, E4) { set(state, 4, &H); };
INSTRUCTION(cb, E5) { set(state, 4, &L); };
INSTRUCTION(cb, E6) { uint8_t tmp = memory_read(HL); set(state, 4, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, E7) { set(state, 4, &A); };

INSTRUCTION(cb, E8) { set(state, 5, &B); };
INSTRUCTION(cb, E9) { set(state, 5, &C); };
INSTRUCTION(cb, EA) { set(state, 5, &D); };
INSTRUCTION(cb, EB) { set(state, 5, &E); };
INSTRUCTION(cb, EC) { set(state, 5, &H); };
INSTRUCTION(cb, ED) { set(state, 5, &L); };
INSTRUCTION(cb, EE) { uint8_t tmp = memory_read(HL); set(state, 5, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, EF) { set(state, 5, &A); };

INSTRUCTION(cb, F0) { set(state, 6, &B); };
INSTRUCTION(cb, F1) { set(state, 6, &C); };
INSTRUCTION(cb, F2) { set(state, 6, &D); };
INSTRUCTION(cb, F3) { set(state, 6, &E); };
INSTRUCTION(cb, F4) { set(state, 6, &H); };
INSTRUCTION(cb, F5) { set(state, 6, &L); };
INSTRUCTION(cb, F6) { uint8_t tmp = memory_read(HL); set(state, 6, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, F7) { set(state, 6, &A); };

INSTRUCTION(cb, F8) { set(state, 7, &B); };
INSTRUCTION(cb, F9) { set(state, 7, &C); };
INSTRUCTION(cb, FA) { set(state, 7, &D); };
INSTRUCTION(cb, FB) { set(state, 7, &E); };
INSTRUCTION(cb, FC) { set(state, 7, &H); };
INSTRUCTION(cb, FD) { set(state, 7, &L); };
INSTRUCTION(cb, FE) { uint8_t tmp = memory_read(HL); set(state, 7, &tmp); memory_write(HL, tmp); };
INSTRUCTION(cb, FF) { set(state, 7, &A); };

/* ----------------------------------------- */

const Branch cb_table[256] = {
	cb00, cb01, cb02, cb03, cb04, cb05, cb06, cb07, cb08, cb09, cb0A, cb0B, cb0C, cb0D, cb0E, cb0F,
	cb10, cb11, cb12, cb13, cb14, cb15, cb16, cb17, cb18, cb19, cb1A, cb1B, cb1C, cb1D, cb1E, cb1F,
	cb20, cb21, cb22, cb23, cb24, cb25, cb26, cb27, cb28, cb29, cb2A, cb2B, cb2C, cb2D, cb2E, cb2F,
	cb30, cb31, cb32, cb33, cb34, cb35, cb36, cb37, cb38, cb39, cb3A, cb3B, cb3C, cb3D, cb3E, cb3F,
	cb40, cb41, cb42, cb43, cb44, cb45, cb46, cb47, cb48, cb49, cb4A, cb4B, cb4C, cb4D, cb4E, cb4F,
	cb50, cb51, cb52, cb53, cb54, cb55, cb56, cb57, cb58, cb59, cb5A, cb5B, cb5C, cb5D, cb5E, cb5F,
	cb60, cb61, cb62, cb63, cb64, cb65, cb66, cb67, cb68, cb69, cb6A, cb6B, cb6C, cb6D, cb6E, cb6F,
	cb70, cb71, cb72, cb73, cb74, cb75, cb76, cb77, cb78, cb79, cb7A, cb7B, cb7C, cb7D, cb7E, cb7F,
	cb80, cb81, cb82, cb83, cb84, cb85, cb86, cb87, cb88, cb89, cb8A, cb8B, cb8C, cb8D, cb8E, cb8F,
	cb90, cb91, cb92, cb93, cb94, cb95, cb96, cb97, cb98, cb99, cb9A, cb9B, cb9C, cb9D, cb9E, cb9F,
	cbA0, cbA1, cbA2, cbA3, cbA4, cbA5, cbA6, cbA7, cbA8, cbA9, cbAA, cbAB, cbAC, cbAD, cbAE, cbAF,
	cbB0, cbB1, cbB2, cbB3, cbB4, cbB5, cbB6, cbB7, cbB8, cbB9, cbBA, cbBB, cbBC, cbBD, cbBE, cbBF,
	cbC0, cbC1, cbC2, cbC3, cbC4, cbC5, cbC6, cbC7, cbC8, cbC9, cbCA, cbCB, cbCC, cbCD, cbCE, cbCF,
	cbD0, cbD1, cbD2, cbD3, cbD4, cbD5, cbD6, cbD7, cbD8, cbD9, cbDA, cbDB, cbDC, cbDD, cbDE, cbDF,
	cbE0, cbE1, cbE2, cbE3, cbE4, cbE5, cbE6, cbE7, cbE8, cbE9, cbEA, cbEB, cbEC, cbED, cbEE, cbEF,
	cbF0, cbF1, cbF2, cbF3, cbF4, cbF5, cbF6, cbF7, cbF8, cbF9, cbFA, cbFB, cbFC, cbFD, cbFE, cbFF
};

void cb_sw(Z80State * state){
	cb_table[fetch_byte(state)](state);
};

/* ----------------------------------------- */

typedef void (*XYCB_Branch)(Z80State * state, uint16_t addr);

XYCB_INSTRUCTION(xycb, 00) { uint8_t tmp = memory_read(addr); rlc(state, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 01) { uint8_t tmp = memory_read(addr); rlc(state, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 02) { uint8_t tmp = memory_read(addr); rlc(state, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 03) { uint8_t tmp = memory_read(addr); rlc(state, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 04) { uint8_t tmp = memory_read(addr); rlc(state, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 05) { uint8_t tmp = memory_read(addr); rlc(state, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 06) { uint8_t tmp = memory_read(addr); rlc(state, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 07) { uint8_t tmp = memory_read(addr); rlc(state, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, 08) { uint8_t tmp = memory_read(addr); rrc(state, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 09) { uint8_t tmp = memory_read(addr); rrc(state, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 0A) { uint8_t tmp = memory_read(addr); rrc(state, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 0B) { uint8_t tmp = memory_read(addr); rrc(state, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 0C) { uint8_t tmp = memory_read(addr); rrc(state, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 0D) { uint8_t tmp = memory_read(addr); rrc(state, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 0E) { uint8_t tmp = memory_read(addr); rrc(state, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 0F) { uint8_t tmp = memory_read(addr); rrc(state, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, 10) { uint8_t tmp = memory_read(addr); rl(state, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 11) { uint8_t tmp = memory_read(addr); rl(state, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 12) { uint8_t tmp = memory_read(addr); rl(state, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 13) { uint8_t tmp = memory_read(addr); rl(state, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 14) { uint8_t tmp = memory_read(addr); rl(state, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 15) { uint8_t tmp = memory_read(addr); rl(state, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 16) { uint8_t tmp = memory_read(addr); rl(state, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 17) { uint8_t tmp = memory_read(addr); rl(state, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, 18) { uint8_t tmp = memory_read(addr); rr(state, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 19) { uint8_t tmp = memory_read(addr); rr(state, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 1A) { uint8_t tmp = memory_read(addr); rr(state, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 1B) { uint8_t tmp = memory_read(addr); rr(state, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 1C) { uint8_t tmp = memory_read(addr); rr(state, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 1D) { uint8_t tmp = memory_read(addr); rr(state, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 1E) { uint8_t tmp = memory_read(addr); rr(state, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 1F) { uint8_t tmp = memory_read(addr); rr(state, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, 20) { uint8_t tmp = memory_read(addr); sla(state, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 21) { uint8_t tmp = memory_read(addr); sla(state, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 22) { uint8_t tmp = memory_read(addr); sla(state, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 23) { uint8_t tmp = memory_read(addr); sla(state, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 24) { uint8_t tmp = memory_read(addr); sla(state, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 25) { uint8_t tmp = memory_read(addr); sla(state, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 26) { uint8_t tmp = memory_read(addr); sla(state, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 27) { uint8_t tmp = memory_read(addr); sla(state, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, 28) { uint8_t tmp = memory_read(addr); sra(state, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 29) { uint8_t tmp = memory_read(addr); sra(state, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 2A) { uint8_t tmp = memory_read(addr); sra(state, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 2B) { uint8_t tmp = memory_read(addr); sra(state, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 2C) { uint8_t tmp = memory_read(addr); sra(state, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 2D) { uint8_t tmp = memory_read(addr); sra(state, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 2E) { uint8_t tmp = memory_read(addr); sra(state, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 2F) { uint8_t tmp = memory_read(addr); sra(state, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, 30) { uint8_t tmp = memory_read(addr); sll(state, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 31) { uint8_t tmp = memory_read(addr); sll(state, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 32) { uint8_t tmp = memory_read(addr); sll(state, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 33) { uint8_t tmp = memory_read(addr); sll(state, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 34) { uint8_t tmp = memory_read(addr); sll(state, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 35) { uint8_t tmp = memory_read(addr); sll(state, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 36) { uint8_t tmp = memory_read(addr); sll(state, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 37) { uint8_t tmp = memory_read(addr); sll(state, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, 38) { uint8_t tmp = memory_read(addr); srl(state, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 39) { uint8_t tmp = memory_read(addr); srl(state, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 3A) { uint8_t tmp = memory_read(addr); srl(state, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 3B) { uint8_t tmp = memory_read(addr); srl(state, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 3C) { uint8_t tmp = memory_read(addr); srl(state, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 3D) { uint8_t tmp = memory_read(addr); srl(state, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 3E) { uint8_t tmp = memory_read(addr); srl(state, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 3F) { uint8_t tmp = memory_read(addr); srl(state, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, 40) { uint8_t tmp = memory_read(addr); bit(state, 0, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 48) { uint8_t tmp = memory_read(addr); bit(state, 1, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 50) { uint8_t tmp = memory_read(addr); bit(state, 2, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 58) { uint8_t tmp = memory_read(addr); bit(state, 3, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 60) { uint8_t tmp = memory_read(addr); bit(state, 4, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 68) { uint8_t tmp = memory_read(addr); bit(state, 5, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 70) { uint8_t tmp = memory_read(addr); bit(state, 6, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 78) { uint8_t tmp = memory_read(addr); bit(state, 7, &tmp); memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, 80) { uint8_t tmp = memory_read(addr); res(state, 0, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 81) { uint8_t tmp = memory_read(addr); res(state, 0, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 82) { uint8_t tmp = memory_read(addr); res(state, 0, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 83) { uint8_t tmp = memory_read(addr); res(state, 0, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 84) { uint8_t tmp = memory_read(addr); res(state, 0, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 85) { uint8_t tmp = memory_read(addr); res(state, 0, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 86) { uint8_t tmp = memory_read(addr); res(state, 0, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 87) { uint8_t tmp = memory_read(addr); res(state, 0, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, 88) { uint8_t tmp = memory_read(addr); res(state, 1, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 89) { uint8_t tmp = memory_read(addr); res(state, 1, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 8A) { uint8_t tmp = memory_read(addr); res(state, 1, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 8B) { uint8_t tmp = memory_read(addr); res(state, 1, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 8C) { uint8_t tmp = memory_read(addr); res(state, 1, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 8D) { uint8_t tmp = memory_read(addr); res(state, 1, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 8E) { uint8_t tmp = memory_read(addr); res(state, 1, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 8F) { uint8_t tmp = memory_read(addr); res(state, 1, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, 90) { uint8_t tmp = memory_read(addr); res(state, 2, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 91) { uint8_t tmp = memory_read(addr); res(state, 2, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 92) { uint8_t tmp = memory_read(addr); res(state, 2, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 93) { uint8_t tmp = memory_read(addr); res(state, 2, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 94) { uint8_t tmp = memory_read(addr); res(state, 2, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 95) { uint8_t tmp = memory_read(addr); res(state, 2, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 96) { uint8_t tmp = memory_read(addr); res(state, 2, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 97) { uint8_t tmp = memory_read(addr); res(state, 2, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, 98) { uint8_t tmp = memory_read(addr); res(state, 3, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 99) { uint8_t tmp = memory_read(addr); res(state, 3, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 9A) { uint8_t tmp = memory_read(addr); res(state, 3, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 9B) { uint8_t tmp = memory_read(addr); res(state, 3, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 9C) { uint8_t tmp = memory_read(addr); res(state, 3, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 9D) { uint8_t tmp = memory_read(addr); res(state, 3, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 9E) { uint8_t tmp = memory_read(addr); res(state, 3, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, 9F) { uint8_t tmp = memory_read(addr); res(state, 3, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, A0) { uint8_t tmp = memory_read(addr); res(state, 4, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, A1) { uint8_t tmp = memory_read(addr); res(state, 4, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, A2) { uint8_t tmp = memory_read(addr); res(state, 4, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, A3) { uint8_t tmp = memory_read(addr); res(state, 4, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, A4) { uint8_t tmp = memory_read(addr); res(state, 4, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, A5) { uint8_t tmp = memory_read(addr); res(state, 4, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, A6) { uint8_t tmp = memory_read(addr); res(state, 4, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, A7) { uint8_t tmp = memory_read(addr); res(state, 4, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, A8) { uint8_t tmp = memory_read(addr); res(state, 5, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, A9) { uint8_t tmp = memory_read(addr); res(state, 5, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, AA) { uint8_t tmp = memory_read(addr); res(state, 5, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, AB) { uint8_t tmp = memory_read(addr); res(state, 5, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, AC) { uint8_t tmp = memory_read(addr); res(state, 5, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, AD) { uint8_t tmp = memory_read(addr); res(state, 5, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, AE) { uint8_t tmp = memory_read(addr); res(state, 5, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, AF) { uint8_t tmp = memory_read(addr); res(state, 5, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, B0) { uint8_t tmp = memory_read(addr); res(state, 6, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, B1) { uint8_t tmp = memory_read(addr); res(state, 6, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, B2) { uint8_t tmp = memory_read(addr); res(state, 6, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, B3) { uint8_t tmp = memory_read(addr); res(state, 6, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, B4) { uint8_t tmp = memory_read(addr); res(state, 6, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, B5) { uint8_t tmp = memory_read(addr); res(state, 6, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, B6) { uint8_t tmp = memory_read(addr); res(state, 6, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, B7) { uint8_t tmp = memory_read(addr); res(state, 6, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, B8) { uint8_t tmp = memory_read(addr); res(state, 7, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, B9) { uint8_t tmp = memory_read(addr); res(state, 7, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, BA) { uint8_t tmp = memory_read(addr); res(state, 7, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, BB) { uint8_t tmp = memory_read(addr); res(state, 7, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, BC) { uint8_t tmp = memory_read(addr); res(state, 7, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, BD) { uint8_t tmp = memory_read(addr); res(state, 7, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, BE) { uint8_t tmp = memory_read(addr); res(state, 7, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, BF) { uint8_t tmp = memory_read(addr); res(state, 7, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, C0) { uint8_t tmp = memory_read(addr); set(state, 0, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, C1) { uint8_t tmp = memory_read(addr); set(state, 0, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, C2) { uint8_t tmp = memory_read(addr); set(state, 0, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, C3) { uint8_t tmp = memory_read(addr); set(state, 0, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, C4) { uint8_t tmp = memory_read(addr); set(state, 0, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, C5) { uint8_t tmp = memory_read(addr); set(state, 0, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, C6) { uint8_t tmp = memory_read(addr); set(state, 0, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, C7) { uint8_t tmp = memory_read(addr); set(state, 0, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, C8) { uint8_t tmp = memory_read(addr); set(state, 1, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, C9) { uint8_t tmp = memory_read(addr); set(state, 1, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, CA) { uint8_t tmp = memory_read(addr); set(state, 1, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, CB) { uint8_t tmp = memory_read(addr); set(state, 1, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, CC) { uint8_t tmp = memory_read(addr); set(state, 1, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, CD) { uint8_t tmp = memory_read(addr); set(state, 1, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, CE) { uint8_t tmp = memory_read(addr); set(state, 1, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, CF) { uint8_t tmp = memory_read(addr); set(state, 1, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, D0) { uint8_t tmp = memory_read(addr); set(state, 2, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, D1) { uint8_t tmp = memory_read(addr); set(state, 2, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, D2) { uint8_t tmp = memory_read(addr); set(state, 2, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, D3) { uint8_t tmp = memory_read(addr); set(state, 2, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, D4) { uint8_t tmp = memory_read(addr); set(state, 2, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, D5) { uint8_t tmp = memory_read(addr); set(state, 2, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, D6) { uint8_t tmp = memory_read(addr); set(state, 2, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, D7) { uint8_t tmp = memory_read(addr); set(state, 2, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, D8) { uint8_t tmp = memory_read(addr); set(state, 3, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, D9) { uint8_t tmp = memory_read(addr); set(state, 3, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, DA) { uint8_t tmp = memory_read(addr); set(state, 3, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, DB) { uint8_t tmp = memory_read(addr); set(state, 3, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, DC) { uint8_t tmp = memory_read(addr); set(state, 3, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, DD) { uint8_t tmp = memory_read(addr); set(state, 3, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, DE) { uint8_t tmp = memory_read(addr); set(state, 3, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, DF) { uint8_t tmp = memory_read(addr); set(state, 3, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, E0) { uint8_t tmp = memory_read(addr); set(state, 4, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, E1) { uint8_t tmp = memory_read(addr); set(state, 4, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, E2) { uint8_t tmp = memory_read(addr); set(state, 4, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, E3) { uint8_t tmp = memory_read(addr); set(state, 4, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, E4) { uint8_t tmp = memory_read(addr); set(state, 4, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, E5) { uint8_t tmp = memory_read(addr); set(state, 4, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, E6) { uint8_t tmp = memory_read(addr); set(state, 4, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, E7) { uint8_t tmp = memory_read(addr); set(state, 4, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, E8) { uint8_t tmp = memory_read(addr); set(state, 5, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, E9) { uint8_t tmp = memory_read(addr); set(state, 5, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, EA) { uint8_t tmp = memory_read(addr); set(state, 5, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, EB) { uint8_t tmp = memory_read(addr); set(state, 5, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, EC) { uint8_t tmp = memory_read(addr); set(state, 5, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, ED) { uint8_t tmp = memory_read(addr); set(state, 5, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, EE) { uint8_t tmp = memory_read(addr); set(state, 5, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, EF) { uint8_t tmp = memory_read(addr); set(state, 5, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, F0) { uint8_t tmp = memory_read(addr); set(state, 6, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, F1) { uint8_t tmp = memory_read(addr); set(state, 6, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, F2) { uint8_t tmp = memory_read(addr); set(state, 6, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, F3) { uint8_t tmp = memory_read(addr); set(state, 6, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, F4) { uint8_t tmp = memory_read(addr); set(state, 6, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, F5) { uint8_t tmp = memory_read(addr); set(state, 6, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, F6) { uint8_t tmp = memory_read(addr); set(state, 6, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, F7) { uint8_t tmp = memory_read(addr); set(state, 6, &tmp); A = tmp; memory_write(addr, tmp); };  

XYCB_INSTRUCTION(xycb, F8) { uint8_t tmp = memory_read(addr); set(state, 7, &tmp); B = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, F9) { uint8_t tmp = memory_read(addr); set(state, 7, &tmp); C = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, FA) { uint8_t tmp = memory_read(addr); set(state, 7, &tmp); D = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, FB) { uint8_t tmp = memory_read(addr); set(state, 7, &tmp); E = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, FC) { uint8_t tmp = memory_read(addr); set(state, 7, &tmp); H = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, FD) { uint8_t tmp = memory_read(addr); set(state, 7, &tmp); L = tmp; memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, FE) { uint8_t tmp = memory_read(addr); set(state, 7, &tmp); memory_write(addr, tmp); };  
XYCB_INSTRUCTION(xycb, FF) { uint8_t tmp = memory_read(addr); set(state, 7, &tmp); A = tmp; memory_write(addr, tmp); };  

/* ----------------------------------------- */

const XYCB_Branch xycb_table[256] = {
	xycb00, xycb01, xycb02, xycb03, xycb04, xycb05, xycb06, xycb07, xycb08, xycb09, xycb0A, xycb0B, xycb0C, xycb0D, xycb0E, xycb0F,
	xycb10, xycb11, xycb12, xycb13, xycb14, xycb15, xycb16, xycb17, xycb18, xycb19, xycb1A, xycb1B, xycb1C, xycb1D, xycb1E, xycb1F,
	xycb20, xycb21, xycb22, xycb23, xycb24, xycb25, xycb26, xycb27, xycb28, xycb29, xycb2A, xycb2B, xycb2C, xycb2D, xycb2E, xycb2F,
	xycb30, xycb31, xycb32, xycb33, xycb34, xycb35, xycb36, xycb37, xycb38, xycb39, xycb3A, xycb3B, xycb3C, xycb3D, xycb3E, xycb3F,
	xycb40, xycb40, xycb40, xycb40, xycb40, xycb40, xycb40, xycb40, xycb48, xycb48, xycb48, xycb48, xycb48, xycb48, xycb48, xycb48,
	xycb50, xycb50, xycb50, xycb50, xycb50, xycb50, xycb50, xycb50, xycb58, xycb58, xycb58, xycb58, xycb58, xycb58, xycb58, xycb58,
	xycb60, xycb60, xycb60, xycb60, xycb60, xycb60, xycb60, xycb60, xycb68, xycb68, xycb68, xycb68, xycb68, xycb68, xycb68, xycb68,
	xycb70, xycb70, xycb70, xycb70, xycb70, xycb70, xycb70, xycb70, xycb78, xycb78, xycb78, xycb78, xycb78, xycb78, xycb78, xycb78,
	xycb80, xycb81, xycb82, xycb83, xycb84, xycb85, xycb86, xycb87, xycb88, xycb89, xycb8A, xycb8B, xycb8C, xycb8D, xycb8E, xycb8F,
	xycb90, xycb91, xycb92, xycb93, xycb94, xycb95, xycb96, xycb97, xycb98, xycb99, xycb9A, xycb9B, xycb9C, xycb9D, xycb9E, xycb9F,
	xycbA0, xycbA1, xycbA2, xycbA3, xycbA4, xycbA5, xycbA6, xycbA7, xycbA8, xycbA9, xycbAA, xycbAB, xycbAC, xycbAD, xycbAE, xycbAF,
	xycbB0, xycbB1, xycbB2, xycbB3, xycbB4, xycbB5, xycbB6, xycbB7, xycbB8, xycbB9, xycbBA, xycbBB, xycbBC, xycbBD, xycbBE, xycbBF,
	xycbC0, xycbC1, xycbC2, xycbC3, xycbC4, xycbC5, xycbC6, xycbC7, xycbC8, xycbC9, xycbCA, xycbCB, xycbCC, xycbCD, xycbCE, xycbCF,
	xycbD0, xycbD1, xycbD2, xycbD3, xycbD4, xycbD5, xycbD6, xycbD7, xycbD8, xycbD9, xycbDA, xycbDB, xycbDC, xycbDD, xycbDE, xycbDF,
	xycbE0, xycbE1, xycbE2, xycbE3, xycbE4, xycbE5, xycbE6, xycbE7, xycbE8, xycbE9, xycbEA, xycbEB, xycbEC, xycbED, xycbEE, xycbEF,
	xycbF0, xycbF1, xycbF2, xycbF3, xycbF4, xycbF5, xycbF6, xycbF7, xycbF8, xycbF9, xycbFA, xycbFB, xycbFC, xycbFD, xycbFE, xycbFF
};

void xycb_sw(Z80State * state, uint16_t * ix_reg){
	LEA;
	//printf("DDFDCB instruction: %X\n", z80_ram[PC]);
	xycb_table[fetch_byte(state)](state, addr);
};

/* ----------------------------------------- */

typedef void (*XY_Branch)(Z80State * state, uint16_t * ix_reg);

XY_INSTRUCTION(ddfd, XX) { };

XY_INSTRUCTION(ddfd, 09) { add_xy_rr(state, ix_reg, &BC); };

XY_INSTRUCTION(ddfd, 19) { add_xy_rr(state, ix_reg, &DE); };

XY_INSTRUCTION(ddfd, 21) { *ix_reg = fetch_word(state); };
XY_INSTRUCTION(ddfd, 22) { memory_write_16(fetch_word(state), *ix_reg); };
XY_INSTRUCTION(ddfd, 23) { (*ix_reg)++; };
XY_INSTRUCTION(ddfd, 24) { inc_r(state, (uint8_t *)ix_reg + 1); };
XY_INSTRUCTION(ddfd, 25) { dec_r(state, (uint8_t *)ix_reg + 1); };
XY_INSTRUCTION(ddfd, 26) { *((uint8_t *)ix_reg + 1) = fetch_byte(state); };

XY_INSTRUCTION(ddfd, 29) { add_xy_rr(state, ix_reg, ix_reg); };
XY_INSTRUCTION(ddfd, 2A) { *ix_reg = memory_read_16(fetch_word(state)); };
XY_INSTRUCTION(ddfd, 2B) { (*ix_reg)--; };
XY_INSTRUCTION(ddfd, 2C) { inc_r(state, (uint8_t *)ix_reg); };
XY_INSTRUCTION(ddfd, 2D) { dec_r(state, (uint8_t *)ix_reg); };
XY_INSTRUCTION(ddfd, 2E) { *((uint8_t *)ix_reg) = fetch_byte(state); };

XY_INSTRUCTION(ddfd, 34) { LEA; uint8_t tmp = memory_read(addr); inc_r(state, &tmp); memory_write(addr, tmp); };
XY_INSTRUCTION(ddfd, 35) { LEA; uint8_t tmp = memory_read(addr); dec_r(state, &tmp); memory_write(addr, tmp); };
XY_INSTRUCTION(ddfd, 36) { LEA; memory_write(addr, fetch_byte(state)); };
XY_INSTRUCTION(ddfd, 39) { add_xy_rr(state, ix_reg, &SP); };

XY_INSTRUCTION(ddfd, 44) { B = *(ix_reg) << 8; };
XY_INSTRUCTION(ddfd, 45) { B = *(ix_reg) & 0xFF; };
XY_INSTRUCTION(ddfd, 46) { LEA; B = memory_read(addr); };
XY_INSTRUCTION(ddfd, 4C) { C = *(ix_reg) << 8; };
XY_INSTRUCTION(ddfd, 4D) { C = *(ix_reg) & 0xFF; };
XY_INSTRUCTION(ddfd, 4E) { LEA; C = memory_read(addr); };

XY_INSTRUCTION(ddfd, 54) { D = *(ix_reg) << 8; };
XY_INSTRUCTION(ddfd, 55) { D = *(ix_reg) & 0xFF; };
XY_INSTRUCTION(ddfd, 56) { LEA; D = memory_read(addr); };
XY_INSTRUCTION(ddfd, 5C) { E = *(ix_reg) << 8; };
XY_INSTRUCTION(ddfd, 5D) { E = *(ix_reg) & 0xFF; };
XY_INSTRUCTION(ddfd, 5E) { LEA; E = memory_read(addr); };

XY_INSTRUCTION(ddfd, 60) { *((uint8_t *)ix_reg + 1) = B; };
XY_INSTRUCTION(ddfd, 61) { *((uint8_t *)ix_reg + 1) = C; };
XY_INSTRUCTION(ddfd, 62) { *((uint8_t *)ix_reg + 1) = D; };
XY_INSTRUCTION(ddfd, 63) { *((uint8_t *)ix_reg + 1) = E; };
XY_INSTRUCTION(ddfd, 64) { };
XY_INSTRUCTION(ddfd, 65) { *((uint8_t *)ix_reg + 1) = *ix_reg & 0xFF; };
XY_INSTRUCTION(ddfd, 66) { LEA; H = memory_read(addr); };
XY_INSTRUCTION(ddfd, 67) { *((uint8_t *)ix_reg + 1) = A; };

XY_INSTRUCTION(ddfd, 68) { *((uint8_t *)ix_reg) = B; };
XY_INSTRUCTION(ddfd, 69) { *((uint8_t *)ix_reg) = C; };
XY_INSTRUCTION(ddfd, 6A) { *((uint8_t *)ix_reg) = D; };
XY_INSTRUCTION(ddfd, 6B) { *((uint8_t *)ix_reg) = E; };
XY_INSTRUCTION(ddfd, 6C) { *((uint8_t *)ix_reg) = *((uint8_t *)ix_reg + 1); };
XY_INSTRUCTION(ddfd, 6D) { };
XY_INSTRUCTION(ddfd, 6E) { LEA; L = memory_read(addr); };

XY_INSTRUCTION(ddfd, 70) { LEA; memory_write(addr, B); };
XY_INSTRUCTION(ddfd, 71) { LEA; memory_write(addr, C); };
XY_INSTRUCTION(ddfd, 72) { LEA; memory_write(addr, D); };
XY_INSTRUCTION(ddfd, 73) { LEA; memory_write(addr, E); };
XY_INSTRUCTION(ddfd, 74) { LEA; memory_write(addr, H); };
XY_INSTRUCTION(ddfd, 75) { LEA; memory_write(addr, L); };
XY_INSTRUCTION(ddfd, 77) { LEA; memory_write(addr, A); };

XY_INSTRUCTION(ddfd, 7C) { A = *(ix_reg) << 8; };
XY_INSTRUCTION(ddfd, 7D) { A = *(ix_reg) & 0xFF; };
XY_INSTRUCTION(ddfd, 7E) { LEA; A = memory_read(addr); };

XY_INSTRUCTION(ddfd, 84) { add_8(state, (uint8_t *)ix_reg + 1); };
XY_INSTRUCTION(ddfd, 85) { add_8(state, (uint8_t *)ix_reg); };
XY_INSTRUCTION(ddfd, 86) { LEA; uint8_t tmp = memory_read(addr); add_8(state, &tmp); memory_write(addr, tmp); };

XY_INSTRUCTION(ddfd, 8C) { adc_8(state, (uint8_t *)ix_reg + 1); };
XY_INSTRUCTION(ddfd, 8D) { adc_8(state, (uint8_t *)ix_reg); };
XY_INSTRUCTION(ddfd, 8E) { LEA; uint8_t tmp = memory_read(addr); adc_8(state, &tmp); memory_write(addr, tmp); };

XY_INSTRUCTION(ddfd, 94) { sub_8(state, (uint8_t *)ix_reg + 1); };
XY_INSTRUCTION(ddfd, 95) { sub_8(state, (uint8_t *)ix_reg); };
XY_INSTRUCTION(ddfd, 96) { LEA; uint8_t tmp = memory_read(addr); sub_8(state, &tmp); memory_write(addr, tmp); };

XY_INSTRUCTION(ddfd, 9C) { sbc_8(state, (uint8_t *)ix_reg + 1); };
XY_INSTRUCTION(ddfd, 9D) { sbc_8(state, (uint8_t *)ix_reg); };
XY_INSTRUCTION(ddfd, 9E) { LEA; uint8_t tmp = memory_read(addr); sbc_8(state, &tmp); memory_write(addr, tmp); };

XY_INSTRUCTION(ddfd, A4) { and_8(state, (uint8_t *)ix_reg + 1); };
XY_INSTRUCTION(ddfd, A5) { and_8(state, (uint8_t *)ix_reg); };
XY_INSTRUCTION(ddfd, A6) { LEA; uint8_t tmp = memory_read(addr); and_8(state, &tmp); memory_write(addr, tmp); };

XY_INSTRUCTION(ddfd, AC) { xor_8(state, (uint8_t *)ix_reg + 1); };
XY_INSTRUCTION(ddfd, AD) { xor_8(state, (uint8_t *)ix_reg); };
XY_INSTRUCTION(ddfd, AE) { LEA; uint8_t tmp = memory_read(addr); xor_8(state, &tmp); memory_write(addr, tmp); };

XY_INSTRUCTION(ddfd, B4) { or_8(state, (uint8_t *)ix_reg + 1); };
XY_INSTRUCTION(ddfd, B5) { or_8(state, (uint8_t *)ix_reg); };
XY_INSTRUCTION(ddfd, B6) { LEA; uint8_t tmp = memory_read(addr); or_8(state, &tmp); memory_write(addr, tmp); };

XY_INSTRUCTION(ddfd, CB) { xycb_sw(state, ix_reg); };

XY_INSTRUCTION(ddfd, BC) { cp_8(state, (uint8_t *)ix_reg + 1); };
XY_INSTRUCTION(ddfd, BD) { cp_8(state, (uint8_t *)ix_reg); };
XY_INSTRUCTION(ddfd, BE) { LEA; uint8_t tmp = memory_read(addr); cp_8(state, &tmp); memory_write(addr, tmp); };

XY_INSTRUCTION(ddfd, E1) { pop(state, ix_reg); };
XY_INSTRUCTION(ddfd, E3) { ex_ptrsp_xy(state, ix_reg); };
XY_INSTRUCTION(ddfd, E5) { push(state, ix_reg); };

XY_INSTRUCTION(ddfd, E9) { PC = *ix_reg; };

XY_INSTRUCTION(ddfd, F9) { SP = *ix_reg; };

/* ----------------------------------------- */

const XY_Branch ddfd_table[256] = {
	ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfd09, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX,
	ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfd19, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX,
	ddfdXX, ddfd21, ddfd22, ddfd23, ddfd24, ddfd25, ddfd26, ddfdXX, ddfdXX, ddfd29, ddfd2A, ddfd2B, ddfd2C, ddfd2D, ddfd2E, ddfdXX,
	ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfd34, ddfd35, ddfd36, ddfdXX, ddfdXX, ddfd39, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX,
	ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfd44, ddfd45, ddfd46, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfd4C, ddfd4D, ddfd4E, ddfdXX,
	ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfd54, ddfd55, ddfd56, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfd5C, ddfd5D, ddfd5E, ddfdXX,
	ddfd60, ddfd61, ddfd62, ddfd63, ddfd64, ddfd65, ddfd66, ddfd67, ddfd68, ddfd69, ddfd6A, ddfd6B, ddfd6C, ddfd6D, ddfd6E, ddfdXX,
	ddfd70, ddfd71, ddfd72, ddfd73, ddfd74, ddfd75, ddfdXX, ddfd77, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfd7C, ddfd7D, ddfd7E, ddfdXX,
	ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfd84, ddfd85, ddfd86, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfd8C, ddfd8D, ddfd8E, ddfdXX,
	ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfd94, ddfd95, ddfd96, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfd9C, ddfd9D, ddfd9E, ddfdXX,
	ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdA4, ddfdA5, ddfdA6, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdAC, ddfdAD, ddfdAE, ddfdXX,
	ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdB4, ddfdB5, ddfdB6, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdBC, ddfdBD, ddfdBE, ddfdXX,
	ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdCB, ddfdXX, ddfdXX, ddfdXX, ddfdXX,
	ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX,
	ddfdXX, ddfdE1, ddfdXX, ddfdE3, ddfdXX, ddfdE5, ddfdXX, ddfdXX, ddfdXX, ddfdE9, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX,
	ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdF9, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX, ddfdXX,
};

/* ----------------------------------------- */

void ddfd_sw(Z80State * state, uint16_t * ix_reg){
	ddfd_table[fetch_byte(state)](state, ix_reg);
};

/* ----------------------------------------- */

INSTRUCTION(op, 00) { nop(); };
INSTRUCTION(op, 01) { BC = fetch_word(state); };
INSTRUCTION(op, 02) { memory_write(BC, A); };
INSTRUCTION(op, 03) { BC++; };
INSTRUCTION(op, 04) { inc_r(state, &B); }
INSTRUCTION(op, 05) { dec_r(state, &B); }
INSTRUCTION(op, 06) { B = fetch_byte(state); };
INSTRUCTION(op, 07) { rlca(state); };

INSTRUCTION(op, 08) { ex_af_af_(state); };
INSTRUCTION(op, 09) { add_hl_ss(state, &BC); };
INSTRUCTION(op, 0A) { A = memory_read(BC); };
INSTRUCTION(op, 0B) { BC--; };
INSTRUCTION(op, 0C) { inc_r(state, &C); };
INSTRUCTION(op, 0D) { dec_r(state, &C); };
INSTRUCTION(op, 0E) { C = fetch_byte(state); };
INSTRUCTION(op, 0F) { rrca(state); };

INSTRUCTION(op, 10) { djnz(state); };
INSTRUCTION(op, 11) { DE = fetch_word(state); };
INSTRUCTION(op, 12) { memory_write(DE, A); };
INSTRUCTION(op, 13) { DE++; };
INSTRUCTION(op, 14) { inc_r(state, &D); }
INSTRUCTION(op, 15) { dec_r(state, &D); }
INSTRUCTION(op, 16) { D = fetch_byte(state); };
INSTRUCTION(op, 17) { rla(state); };

INSTRUCTION(op, 18) { int8_t e = fetch_byte(state); PC += e - 1; };
INSTRUCTION(op, 19) { add_hl_ss(state, &DE); };
INSTRUCTION(op, 1A) { A = memory_read(DE); };
INSTRUCTION(op, 1B) { DE--; };
INSTRUCTION(op, 1C) { inc_r(state, &E); };
INSTRUCTION(op, 1D) { dec_r(state, &E); };
INSTRUCTION(op, 1E) { E = fetch_byte(state); };
INSTRUCTION(op, 1F) { rra(state); };

INSTRUCTION(op, 20) { jr_cond(state, !(F & F_Z)); };
INSTRUCTION(op, 21) { HL = fetch_word(state); };
INSTRUCTION(op, 22) { memory_write_16(fetch_word(state), HL); };
INSTRUCTION(op, 23) { HL++; };
INSTRUCTION(op, 24) { inc_r(state, &H); }
INSTRUCTION(op, 25) { dec_r(state, &H); }
INSTRUCTION(op, 26) { H = fetch_byte(state); };
INSTRUCTION(op, 27) { daa(state); };

INSTRUCTION(op, 28) { jr_cond(state, (F & F_Z)); };
INSTRUCTION(op, 29) { add_hl_ss(state, &HL); };
INSTRUCTION(op, 2A) { HL = memory_read_16(fetch_word(state)); };
INSTRUCTION(op, 2B) { HL--; };
INSTRUCTION(op, 2C) { inc_r(state, &L); };
INSTRUCTION(op, 2D) { dec_r(state, &L); };
INSTRUCTION(op, 2E) { L = fetch_byte(state); };
INSTRUCTION(op, 2F) { cpl(state); };

INSTRUCTION(op, 30) { jr_cond(state, !(F & F_C)); };
INSTRUCTION(op, 31) { SP = fetch_word(state); };
INSTRUCTION(op, 32) { memory_write(fetch_word(state), A); };
INSTRUCTION(op, 33) { SP++; };
INSTRUCTION(op, 34) { inc_r(state, z80_ram + HL); }
INSTRUCTION(op, 35) { dec_r(state, z80_ram + HL); }
INSTRUCTION(op, 36) { uint8_t tmp = fetch_byte(state); memory_write(HL, tmp); };
INSTRUCTION(op, 37) { scf(state); };

INSTRUCTION(op, 38) { jr_cond(state, (F & F_C)); };
INSTRUCTION(op, 39) { add_hl_ss(state, &SP); };
INSTRUCTION(op, 3A) { A = memory_read(fetch_word(state)); };
INSTRUCTION(op, 3B) { SP--; };
INSTRUCTION(op, 3C) { inc_r(state, &A); };
INSTRUCTION(op, 3D) { dec_r(state, &A); };
INSTRUCTION(op, 3E) { A = fetch_byte(state); };
INSTRUCTION(op, 3F) { ccf(state); };

INSTRUCTION(op, 40) { };
INSTRUCTION(op, 41) { B = C; };
INSTRUCTION(op, 42) { B = D; };
INSTRUCTION(op, 43) { B = E; };
INSTRUCTION(op, 44) { B = H; };
INSTRUCTION(op, 45) { B = L; };
INSTRUCTION(op, 46) { B = memory_read(HL); };
INSTRUCTION(op, 47) { B = A; };

INSTRUCTION(op, 48) { C = B; };
INSTRUCTION(op, 49) { };
INSTRUCTION(op, 4A) { C = D; };
INSTRUCTION(op, 4B) { C = E; };
INSTRUCTION(op, 4C) { C = H; };
INSTRUCTION(op, 4D) { C = L; };
INSTRUCTION(op, 4E) { C = memory_read(HL); };
INSTRUCTION(op, 4F) { C = A; };

INSTRUCTION(op, 50) { D = B; };
INSTRUCTION(op, 51) { D = C; };
INSTRUCTION(op, 52) { };
INSTRUCTION(op, 53) { D = E; };
INSTRUCTION(op, 54) { D = H; };
INSTRUCTION(op, 55) { D = L; };
INSTRUCTION(op, 56) { D = memory_read(HL); };
INSTRUCTION(op, 57) { D = A; };

INSTRUCTION(op, 58) { E = B; };
INSTRUCTION(op, 59) { E = C; };
INSTRUCTION(op, 5A) { E = D; };
INSTRUCTION(op, 5B) { };
INSTRUCTION(op, 5C) { E = H; };
INSTRUCTION(op, 5D) { E = L; };
INSTRUCTION(op, 5E) { E = memory_read(HL); };
INSTRUCTION(op, 5F) { E = A; };

INSTRUCTION(op, 60) { H = B; };
INSTRUCTION(op, 61) { H = C; };
INSTRUCTION(op, 62) { H = D; };
INSTRUCTION(op, 63) { H = E; };
INSTRUCTION(op, 64) { };
INSTRUCTION(op, 65) { H = L; };
INSTRUCTION(op, 66) { H = memory_read(HL); };
INSTRUCTION(op, 67) { H = A; };

INSTRUCTION(op, 68) { L = B; };
INSTRUCTION(op, 69) { L = C; };
INSTRUCTION(op, 6A) { L = D; };
INSTRUCTION(op, 6B) { L = E; };
INSTRUCTION(op, 6C) { L = H; };
INSTRUCTION(op, 6D) { };
INSTRUCTION(op, 6E) { L = memory_read(HL); };
INSTRUCTION(op, 6F) { L = A; };

INSTRUCTION(op, 70) { memory_write(HL, B); };
INSTRUCTION(op, 71) { memory_write(HL, C); };
INSTRUCTION(op, 72) { memory_write(HL, D); };
INSTRUCTION(op, 73) { memory_write(HL, E); };
INSTRUCTION(op, 74) { memory_write(HL, H); };
INSTRUCTION(op, 75) { memory_write(HL, L); };
INSTRUCTION(op, 76) { state->halt = 1; };
INSTRUCTION(op, 77) { memory_write(HL, A); };

INSTRUCTION(op, 78) { A = B; };
INSTRUCTION(op, 79) { A = C; };
INSTRUCTION(op, 7A) { A = D; };
INSTRUCTION(op, 7B) { A = E; };
INSTRUCTION(op, 7C) { A = H; };
INSTRUCTION(op, 7D) { A = L; };
INSTRUCTION(op, 7E) { A = memory_read(HL); };
INSTRUCTION(op, 7F) { };

INSTRUCTION(op, 80) { add_8(state, &B); };
INSTRUCTION(op, 81) { add_8(state, &C); };
INSTRUCTION(op, 82) { add_8(state, &D); };
INSTRUCTION(op, 83) { add_8(state, &E); };
INSTRUCTION(op, 84) { add_8(state, &H); };
INSTRUCTION(op, 85) { add_8(state, &L); };
INSTRUCTION(op, 86) { add_8(state, z80_ram + HL); };
INSTRUCTION(op, 87) { add_8(state, &A); };

INSTRUCTION(op, 88) { adc_8(state, &B); };
INSTRUCTION(op, 89) { adc_8(state, &C); };
INSTRUCTION(op, 8A) { adc_8(state, &D); };
INSTRUCTION(op, 8B) { adc_8(state, &E); };
INSTRUCTION(op, 8C) { adc_8(state, &H); };
INSTRUCTION(op, 8D) { adc_8(state, &L); };
INSTRUCTION(op, 8E) { adc_8(state, z80_ram + HL); };
INSTRUCTION(op, 8F) { adc_8(state, &A); };

INSTRUCTION(op, 90) { sub_8(state, &B); };
INSTRUCTION(op, 91) { sub_8(state, &C); };
INSTRUCTION(op, 92) { sub_8(state, &D); };
INSTRUCTION(op, 93) { sub_8(state, &E); };
INSTRUCTION(op, 94) { sub_8(state, &H); };
INSTRUCTION(op, 95) { sub_8(state, &L); };
INSTRUCTION(op, 96) { sub_8(state, z80_ram + HL); };
INSTRUCTION(op, 97) { sub_8(state, &A); };

INSTRUCTION(op, 98) { sbc_8(state, &B); };
INSTRUCTION(op, 99) { sbc_8(state, &C); };
INSTRUCTION(op, 9A) { sbc_8(state, &D); };
INSTRUCTION(op, 9B) { sbc_8(state, &E); };
INSTRUCTION(op, 9C) { sbc_8(state, &H); };
INSTRUCTION(op, 9D) { sbc_8(state, &L); };
INSTRUCTION(op, 9E) { sbc_8(state, z80_ram + HL); };
INSTRUCTION(op, 9F) { sbc_8(state, &A); };

INSTRUCTION(op, A0) { and_8(state, &B); };
INSTRUCTION(op, A1) { and_8(state, &C); };
INSTRUCTION(op, A2) { and_8(state, &D); };
INSTRUCTION(op, A3) { and_8(state, &E); };
INSTRUCTION(op, A4) { and_8(state, &H); };
INSTRUCTION(op, A5) { and_8(state, &L); };
INSTRUCTION(op, A6) { and_8(state, z80_ram + HL); };
INSTRUCTION(op, A7) { and_8(state, &A); };

INSTRUCTION(op, A8) { xor_8(state, &B); };
INSTRUCTION(op, A9) { xor_8(state, &C); };
INSTRUCTION(op, AA) { xor_8(state, &D); };
INSTRUCTION(op, AB) { xor_8(state, &E); };
INSTRUCTION(op, AC) { xor_8(state, &H); };
INSTRUCTION(op, AD) { xor_8(state, &L); };
INSTRUCTION(op, AE) { xor_8(state, z80_ram + HL); };
INSTRUCTION(op, AF) { xor_8(state, &A); };

INSTRUCTION(op, B0) { or_8(state, &B); };
INSTRUCTION(op, B1) { or_8(state, &C); };
INSTRUCTION(op, B2) { or_8(state, &D); };
INSTRUCTION(op, B3) { or_8(state, &E); };
INSTRUCTION(op, B4) { or_8(state, &H); };
INSTRUCTION(op, B5) { or_8(state, &L); };
INSTRUCTION(op, B6) { or_8(state, z80_ram + HL); };
INSTRUCTION(op, B7) { or_8(state, &A); };

INSTRUCTION(op, B8) { cp_8(state, &B); };
INSTRUCTION(op, B9) { cp_8(state, &C); };
INSTRUCTION(op, BA) { cp_8(state, &D); };
INSTRUCTION(op, BB) { cp_8(state, &E); };
INSTRUCTION(op, BC) { cp_8(state, &H); };
INSTRUCTION(op, BD) { cp_8(state, &L); };
INSTRUCTION(op, BE) { cp_8(state, z80_ram + HL); };
INSTRUCTION(op, BF) { cp_8(state, &A); };

INSTRUCTION(op, C0) { ret_cond(state, !(F & F_Z)); };
INSTRUCTION(op, C1) { pop(state, &BC); };
INSTRUCTION(op, C2) { jp_cond(state, !(F & F_Z)); };
INSTRUCTION(op, C3) { jp(state); };
INSTRUCTION(op, C4) { call_cond(state, !(F & F_Z)); };
INSTRUCTION(op, C5) { push(state, &BC); };
INSTRUCTION(op, C6) { uint8_t tmp = fetch_byte(state); add_8(state, &tmp); };
INSTRUCTION(op, C7) { rst(state, 0x00); };

INSTRUCTION(op, C8) { ret_cond(state, (F & F_Z)); };
INSTRUCTION(op, C9) { ret(state); };
INSTRUCTION(op, CA) { jp_cond(state, (F & F_Z)); };
INSTRUCTION(op, CB) { cb_sw(state); };
INSTRUCTION(op, CC) { call_cond(state, (F & F_Z)); };
INSTRUCTION(op, CD) { call(state); };
INSTRUCTION(op, CE) { uint8_t tmp = fetch_byte(state); adc_8(state, &tmp); };
INSTRUCTION(op, CF) { rst(state, 0x08); };

INSTRUCTION(op, D0) { ret_cond(state, !(F & F_C)); };
INSTRUCTION(op, D1) { pop(state, &DE); };
INSTRUCTION(op, D2) { jp_cond(state, !(F & F_C)); };
INSTRUCTION(op, D3) { TO_FILL(); }; // TODO!!!!
INSTRUCTION(op, D4) { call_cond(state, !(F & F_C)); };
INSTRUCTION(op, D5) { push(state, &DE); };
INSTRUCTION(op, D6) { uint8_t tmp = fetch_byte(state); sub_8(state, &tmp); };
INSTRUCTION(op, D7) { rst(state, 0x10); };

INSTRUCTION(op, D8) { ret_cond(state, (F & F_C)); };
INSTRUCTION(op, D9) { exx(state); };
INSTRUCTION(op, DA) { jp_cond(state, (F & F_C)); };
INSTRUCTION(op, DB) { TO_FILL(); }; // TODO!!!
INSTRUCTION(op, DC) { call_cond(state, (F & F_C)); };
INSTRUCTION(op, DD) { ddfd_sw(state, &IX); };
INSTRUCTION(op, DE) { uint8_t tmp = fetch_byte(state); sbc_8(state, &tmp); };
INSTRUCTION(op, DF) { rst(state, 0x18); };

INSTRUCTION(op, E0) { ret_cond(state, !(F & F_P)); };
INSTRUCTION(op, E1) { pop(state, &HL); };
INSTRUCTION(op, E2) { jp_cond(state, !(F & F_P)); };
INSTRUCTION(op, E3) { ex_ptrsp_hl(state); }; 
INSTRUCTION(op, E4) { call_cond(state, !(F & F_P)); };
INSTRUCTION(op, E5) { push(state, &HL); };
INSTRUCTION(op, E6) { uint8_t tmp = fetch_byte(state); and_8(state, &tmp); };
INSTRUCTION(op, E7) { rst(state, 0x20); };

INSTRUCTION(op, E8) { ret_cond(state, (F & F_P)); };
INSTRUCTION(op, E9) { PC = HL; };
INSTRUCTION(op, EA) { jp_cond(state, (F & F_P)); };
INSTRUCTION(op, EB) { ex_de_hl(state); }; 
INSTRUCTION(op, EC) { call_cond(state, (F & F_P)); };
INSTRUCTION(op, ED) { ed_sw(state); };
INSTRUCTION(op, EE) { uint8_t tmp = fetch_byte(state); xor_8(state, &tmp); };
INSTRUCTION(op, EF) { rst(state, 0x28); };

INSTRUCTION(op, F0) { ret_cond(state, !(F & F_S)); };
INSTRUCTION(op, F1) { pop(state, &AF); };
INSTRUCTION(op, F2) { jp_cond(state, !(F & F_S)); };
INSTRUCTION(op, F3) { state->iff1 = state->iff2 = 0; };
INSTRUCTION(op, F4) { call_cond(state, !(F & F_S)); };
INSTRUCTION(op, F5) { push(state, &AF); };
INSTRUCTION(op, F6) { uint8_t tmp = fetch_byte(state); or_8(state, &tmp); };
INSTRUCTION(op, F7) { rst(state, 0x30); };

INSTRUCTION(op, F8) { ret_cond(state, (F & F_S)); };
INSTRUCTION(op, F9) { SP = HL; };
INSTRUCTION(op, FA) { jp_cond(state, (F & F_S)); };
INSTRUCTION(op, FB) { state->iff1 = state->iff2 = 1; }; 
INSTRUCTION(op, FC) { call_cond(state, (F & F_S)); };
INSTRUCTION(op, FD) { ddfd_sw(state, &IY); };
INSTRUCTION(op, FE) { uint8_t tmp = fetch_byte(state); cp_8(state, &tmp); };
INSTRUCTION(op, FF) { rst(state, 0x38); };

/* ----------------------------------------- */

const Branch branch_table[256] = {
	op00, op01, op02, op03, op04, op05, op06, op07, op08, op09, op0A, op0B, op0C, op0D, op0E, op0F,
	op10, op11, op12, op13, op14, op15, op16, op17, op18, op19, op1A, op1B, op1C, op1D, op1E, op1F,
	op20, op21, op22, op23, op24, op25, op26, op27, op28, op29, op2A, op2B, op2C, op2D, op2E, op2F,
	op30, op31, op32, op33, op34, op35, op36, op37, op38, op39, op3A, op3B, op3C, op3D, op3E, op3F,
	op40, op41, op42, op43, op44, op45, op46, op47, op48, op49, op4A, op4B, op4C, op4D, op4E, op4F,
	op50, op51, op52, op53, op54, op55, op56, op57, op58, op59, op5A, op5B, op5C, op5D, op5E, op5F,
	op60, op61, op62, op63, op64, op65, op66, op67, op68, op69, op6A, op6B, op6C, op6D, op6E, op6F,
	op70, op71, op72, op73, op74, op75, op76, op77, op78, op79, op7A, op7B, op7C, op7D, op7E, op7F,
	op80, op81, op82, op83, op84, op85, op86, op87, op88, op89, op8A, op8B, op8C, op8D, op8E, op8F,
	op90, op91, op92, op93, op94, op95, op96, op97, op98, op99, op9A, op9B, op9C, op9D, op9E, op9F,
	opA0, opA1, opA2, opA3, opA4, opA5, opA6, opA7, opA8, opA9, opAA, opAB, opAC, opAD, opAE, opAF,
	opB0, opB1, opB2, opB3, opB4, opB5, opB6, opB7, opB8, opB9, opBA, opBB, opBC, opBD, opBE, opBF,
	opC0, opC1, opC2, opC3, opC4, opC5, opC6, opC7, opC8, opC9, opCA, opCB, opCC, opCD, opCE, opCF,
	opD0, opD1, opD2, opD3, opD4, opD5, opD6, opD7, opD8, opD9, opDA, opDB, opDC, opDD, opDE, opDF,
	opE0, opE1, opE2, opE3, opE4, opE5, opE6, opE7, opE8, opE9, opEA, opEB, opEC, opED, opEE, opEF,
	opF0, opF1, opF2, opF3, opF4, opF5, opF6, opF7, opF8, opF9, opFA, opFB, opFC, opFD, opFE, opFF
};

/* ----------------------------------------- */

size_t copy_program_to_memory(uint8_t * memory, char * filename){
	if (filename == NULL)
		printf("Invalid ROM filename. Did you forget to load a ROM?");
	FILE * file = fopen(filename, "r");

	fseek (file, 0, SEEK_END);
	long program_size = ftell (file);
	rewind (file);
	if (program_size > 65536)
		printf("Program too big to fit in memory!");

	uint8_t * program_mem = memory + 0x0100; 
	size_t result = fread(program_mem, 1, program_size, file);
	fclose(file);
	return result;
}

void bdos(Z80State * state){ // Emulate CP/M syscalls
	char * ptr;
	switch(state->c){
		case 0x002:
			printf("%c", state->e);
			break;
		case 0x009: 
			ptr = (char *)z80_ram + (uint16_t)state->de;
			while(*ptr != '$'){
				printf("%c", *ptr);
				ptr++;
			}
			break;

	}
}

void zx_print(Z80State * state){
	printf("%c", z80_state->a);
}

void update_ui(uint8_t opcode){
	char tmp[5];

	UPDATE_UI_ELEMENT_8BIT(register_a, a);UPDATE_UI_ELEMENT_8BIT(register_a_, a_);
	UPDATE_UI_ELEMENT_8BIT(register_f, f);UPDATE_UI_ELEMENT_8BIT(register_f_, f_);
	UPDATE_UI_ELEMENT_8BIT(register_b, b);UPDATE_UI_ELEMENT_8BIT(register_b_, b_);
	UPDATE_UI_ELEMENT_8BIT(register_c, c);UPDATE_UI_ELEMENT_8BIT(register_c_, c_);
	UPDATE_UI_ELEMENT_8BIT(register_d, d);UPDATE_UI_ELEMENT_8BIT(register_d_, d_);
	UPDATE_UI_ELEMENT_8BIT(register_e, e);UPDATE_UI_ELEMENT_8BIT(register_e_, e_);
	UPDATE_UI_ELEMENT_8BIT(register_h, h);UPDATE_UI_ELEMENT_8BIT(register_h_, h_);
	UPDATE_UI_ELEMENT_8BIT(register_l, l);UPDATE_UI_ELEMENT_8BIT(register_l_, l_);

	UPDATE_UI_ELEMENT_8BIT(register_i, i);
	UPDATE_UI_ELEMENT_8BIT(register_r, r);

	UPDATE_UI_ELEMENT_16BIT(register_pc, pc);
	UPDATE_UI_ELEMENT_16BIT(register_sp, sp);
	UPDATE_UI_ELEMENT_16BIT(register_ix, ix);
	UPDATE_UI_ELEMENT_16BIT(register_iy, iy);

	snprintf(tmp, 3, "%.2X", opcode);
	gtk_entry_set_text(last_instruction, tmp);

}

void emulation_loop(Z80State * state){
	FILE *fptr = fopen("dump", "w");
	char tmp[150];
	uint8_t opcode;
	while (1){
		opcode = z80_ram[state->pc];
		/*sprintf(tmp, "PC: %X A: %X F: %X B: %X C: %X D: %X E: %X H: %X L: %X SP: %X IX: %X IY: %X (HL): %X | Current instruction: %X\n", state->pc, state->a, state->f, state->b, state->c, state->d, state->e, state->h, state->l, state->sp, state->ix, state->iy, z80_ram[state->hl], opcode);
		fprintf(fptr, "%s", tmp);*/
		state->pc++;
		cycle_count++;
		
		branch_table[opcode](state);
		
		//update_ui(opcode);

		if(state->pc == 0x0005)
			bdos(state);
		else if (state->pc == 0x0000 || state->pc >= 0xFFFF){
			break;
		}
		/*if (cycle_count >= 100000)
			break;*/
	}
	fclose(fptr);
	printf("End dump\n");
}

void gui_connect_fields(GtkBuilder * builder){
	
	register_a = GTK_ENTRY(gtk_builder_get_object(builder, "register_a"));
	register_f = GTK_ENTRY(gtk_builder_get_object(builder, "register_f"));
	register_b = GTK_ENTRY(gtk_builder_get_object(builder, "register_b"));
	register_c = GTK_ENTRY(gtk_builder_get_object(builder, "register_c"));
	register_d = GTK_ENTRY(gtk_builder_get_object(builder, "register_d"));
	register_e = GTK_ENTRY(gtk_builder_get_object(builder, "register_e"));
	register_h = GTK_ENTRY(gtk_builder_get_object(builder, "register_h"));
	register_l = GTK_ENTRY(gtk_builder_get_object(builder, "register_l"));

	register_a_ = GTK_ENTRY(gtk_builder_get_object(builder, "register_a_"));
	register_f_ = GTK_ENTRY(gtk_builder_get_object(builder, "register_f_"));
	register_b_ = GTK_ENTRY(gtk_builder_get_object(builder, "register_b_"));
	register_c_ = GTK_ENTRY(gtk_builder_get_object(builder, "register_c_"));
	register_d_ = GTK_ENTRY(gtk_builder_get_object(builder, "register_d_"));
	register_e_ = GTK_ENTRY(gtk_builder_get_object(builder, "register_e_"));
	register_h_ = GTK_ENTRY(gtk_builder_get_object(builder, "register_h_"));
	register_l_ = GTK_ENTRY(gtk_builder_get_object(builder, "register_l_"));

	register_i = GTK_ENTRY(gtk_builder_get_object(builder, "register_i"));
	register_r = GTK_ENTRY(gtk_builder_get_object(builder, "register_r"));

	register_ix = GTK_ENTRY(gtk_builder_get_object(builder, "register_ix"));
	register_iy = GTK_ENTRY(gtk_builder_get_object(builder, "register_iy"));

	iff1 = GTK_ENTRY(gtk_builder_get_object(builder, "iff1"));
	iff2 = GTK_ENTRY(gtk_builder_get_object(builder, "iff2"));

	register_pc = GTK_ENTRY(gtk_builder_get_object(builder, "register_pc"));
	register_sp = GTK_ENTRY(gtk_builder_get_object(builder, "register_sp"));

	last_instruction = GTK_ENTRY(gtk_builder_get_object(builder, "last_instruction"));

}

void * emulation_init(){	
	copy_program_to_memory(z80_ram, "zexdoc.com");
	z80_state->sp = 0xFFFF;
	z80_state->pc = 0x0100;
	z80_ram[0] = 0xD3;
	z80_ram[5] = 0xDB;
	z80_ram[7] = 0xC9;
	emulation_loop(z80_state);
}

void gui_init(){
	GtkWidget *window;
	GtkBuilder *builder = NULL;
	gtk_init(NULL, NULL);

	builder = gtk_builder_new();
	gtk_builder_add_from_file(builder, "z80_layout.glade", NULL);

	window = GTK_WIDGET(gtk_builder_get_object(builder, "window"));
	gui_connect_fields(builder);
	gtk_builder_connect_signals(builder, NULL);

	gtk_widget_show_all(window);
	pthread_create(&emulation_thread, NULL, emulation_init, NULL);
	gtk_main();
}

int main(){
	gui_init();
}
