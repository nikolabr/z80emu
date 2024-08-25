#ifndef Z80_H
#define Z80_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct { 

	uint8_t i; // Interrupt vector
	uint8_t r; // Memory refresh

	// Index registers
	
	union {
		uint16_t ix; 
		struct{ 
			uint8_t ixl;
			uint8_t ixh;
		};
	};

	union { 
		uint16_t iy;
		struct{ 
			uint8_t iyl;
			uint8_t iyh;
		};
	};

	uint16_t sp; // Stack pointer (points to the top of the stack)
	uint16_t pc; // Program counter


	// Logical accumulator and flag register - main set
	
	union { 
		uint16_t af; 
		struct { 
			uint8_t f;
			uint8_t a;
		};
	};

	// General purpose registers - main set
	
	union { 
		uint16_t bc; 
		struct { 
			uint8_t c;
			uint8_t b;
		};
	};

	union { 
		uint16_t de; 
		struct { 
			uint8_t e;
			uint8_t d;
		};
	};

	union { 
		uint16_t hl; 
		struct { 
			uint8_t l;
			uint8_t h;
		};
	};

	// Logical accumulator and flag register - alternate set

	union { 
		uint16_t af_; 
		struct { 
			uint8_t f_;
			uint8_t a_;
		};
	};
	
	// General purpose registers - alternate set
	
	union { 
		uint16_t bc_; 
		struct { 
			uint8_t c_;
			uint8_t b_;
		};
	};

	union { 
		uint16_t de_; 
		struct { 
			uint8_t e_;
			uint8_t d_;
		};
	};

	union { 
		uint16_t hl_; 
		struct { 
			uint8_t l_;
			uint8_t h_;
		};
	};
	
	// Interrupt flags

	uint8_t iff1 : 1;
	uint8_t iff2 : 1;

	uint8_t mreq : 1;
	uint8_t rd : 1;
	uint8_t rfsh : 1;

	uint8_t halt : 1;
	uint8_t im;

} Z80State;


typedef struct { 
	uint16_t addr_bus;
	uint8_t data_bus;

	void *context;

	Z80State state; 
} Z80;

#endif
