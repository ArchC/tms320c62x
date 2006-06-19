/**
 * @file      tms62x-isa.cpp
 * @author    Sandro Rigo
 *
 *            The ArchC Team
 *            http://www.archc.org/
 *
 *            Computer Systems Laboratory (LSC)
 *            IC-UNICAMP
 *            http://www.lsc.ic.unicamp.br
 *
 * @version   version?
 * @date      Mon, 19 Jun 2006 15:33:29 -0300
 * 
 * @brief     The ArchC TMS320C62x DSP model.
 * 
 * @attention Copyright (C) 2002-2006 --- The ArchC Team
 *
 */
 

#include  "tmsc62x-isa.H"
#include  "ac_isa_init.cpp"
 
//Defining Control Registers names
#define AMR  0
#define CSR  1
#define IFR  2
#define ISR  2
#define ICR  3
#define IER  4
#define ISTP 5
#define IRP  6
#define NRP  7
#define PCE1 16

//Maximum and minimum constants for 40-bit long data type
#define MAX_LONG 0x7FFFFFFFFF   
#define MIN_LONG 0x8000000000   

//Setting the saturation bit in the CSR register.
//When an instruction does saturation, it has to set the bit 9 of the CSR register
#define SATURATE RB_C.write( CSR, ( RB_C.read(CSR) | 0x200 ))

//Debugging function
#define DEBUG
#ifdef DEBUG
#include <stdarg.h>
inline int dprintf(const char *format, ...)
{
  int ret;

	va_list args;
	va_start(args, format);
	ret = vfprintf(ac_err, format, args);
	va_end(args);

  return ret;
}
#else
inline void dprintf(const char *format, ...) {}
#endif

//User defined Variables
long long cycle_count=0;


int xsrc1;  //The cross loaded source operand
int xsrc2;  //The cross loaded source operand
unsigned int bksize;     //The value stored in the bk field of the AMR register to be used for the next 
            //instruction.
int address; //Used by load/store instructions

/*--------------------------------------------------------------------------------*/
//User defined functions

/*This function writes a destination register, checking for
  the correct register bank.*/
void writeReg( int s, int dst, int value ){

	if (s ){
		ac_resources::RB_B.write(dst, value );
		dprintf("Result = %#x (RB_B)\n", ac_resources::RB_B.read(dst));
	}
	else{
		ac_resources::RB_A.write(dst, value );
		dprintf("Result = %#x (RB_A)\n", ac_resources::RB_A.read(dst));
	}
}

/* Writes a 40-bit long value to a pair of registers */
void writeLong(int s, int dst, long long value ){
	
	int msb, lsb;  //Most and less significant bits of a 40-bit long value.

	lsb = (value & 0xFFFFFFFF) ;
	msb = ((value >> 32) & 0xFFFFFFFF);
	msb = msb & 0x000000FF;

	if (s ){
		ac_resources::RB_B.write(dst, lsb );
		ac_resources::RB_B.write(dst+1, msb );
		dprintf("Result = %#x (RB_B(%d))\n", ac_resources::RB_B.read(dst), dst);
		dprintf("Result = %#x (RB_B(%d))\n", ac_resources::RB_B.read(dst+1), dst+1);
	}
	else{
		ac_resources::RB_A.write(dst, lsb );
		ac_resources::RB_A.write(dst+1, msb );
		dprintf("Result = %#x (RB_A(%d))\n", ac_resources::RB_A.read(dst), dst);
		dprintf("Result = %#x (RB_A(%d))\n", ac_resources::RB_A.read(dst+1), dst+1);
	}	
}


/*This function reads a source register, checking for
  the correct register bank.*/
int readReg( int s, int src ){

	if (s ){
		return (int)ac_resources::RB_B.read(src);
	}
	else{
		return (int)ac_resources::RB_A.read(src);
	}
}

/* Reads a 40-bit long value */
long long readLong( int s, int src) {

	long long lsrc=0;
	int msb, lsb;  //Most and less significant bits of a 40-bit long value.

	if(s){
		lsb = ac_resources::RB_B.read(src);
		msb = ac_resources::RB_B.read(src +1);
	}
	else{
		lsb = ac_resources::RB_A.read(src);
		msb = ac_resources::RB_A.read(src +1);
	}

	msb = msb & 0x000000FF;
	lsrc = msb;
	lsrc <<= (long long)32;
	lsrc = lsrc | 0xFFFFFFFF;
	lsrc = lsrc & lsb;

	return lsrc;
}


/* Checks the AMR register to determine which address mode must be used and compute the block size.

   Returns 0 for linear mode and 1 for circular mode. The block size is stored into the bksize
   global variable.
*/
int checkAMR(int s, int reg){

	sc_int<32> amr;
	int modefield;
	int shift;  //Used to adjust the pointer to the correct mode field.

	amr = ac_resources::RB_C.read(AMR);

	//Mode fields for Reg Bank B start on bit 8, and for Reg Bank A they start on bit 0. The s 
	//field passed by the instruction is used to select the correct bank.

	if(s)
		shift = 8;  //Using register bank B
	else
		shift = 0;  //Using register bank A

	switch(reg){
	case 4:			
		modefield = amr.range(1+shift, 0+shift);
		break;
	case 5:
		modefield = amr.range(3+shift, 2+shift);
		break;
	case 6:
		modefield = amr.range(5+shift, 4+shift);
		break;
	case 7:
		modefield = amr.range(7+shift, 6+shift);
		break;
	default:
		cout << "ERROR. Invalid register used for addressing mode computation: " << reg<< endl;
	}

	//Checking the address mode 
	switch( modefield  ){
			
	case 0:
		//Linear
		bksize = 0;
		return 0;
		break;
	case 1:
		//Circular using the BK0 field

		//Computing blocksize
		bksize = 2 << ((unsigned)amr.range(20,16))+1;
		return 1;
		break;
	case 2:
		//Circular using the BK1 field
		bksize = 2 << ((unsigned)amr.range(25,21))+1;
		return 1;
		break;
	default:
		cout << "CheckAMR ERROR! Invalid value into the mode selection field: " << modefield << endl;
		exit(1);
		break;
	}
}

/* Circular addressing mode */
inline int circular(int operand){
	
	//Circular Addressing mode
	dprintf("Circular Addressing mode. Block size: \n", bksize);
	if( (unsigned)operand > bksize ){
		//So, operand is  operand(mod bksize)
		operand = operand % bksize;
	}

	return operand;
}

/* Used by load store to scale the offset accordingly with the data type */
inline int scale_offset(int ld_st, int offset){
	
	if( ld_st == 0x00 || ld_st == 0x04 || ld_st == 0x05){
		//This is a half-word instruction
		offset <<=1;
	}
	else if( ld_st == 0x06 || ld_st == 0x07){
		//This is a word instruction
		offset <<=2;
	}

	return offset;
}

/*--------------------------------------------------------------------------------*/
 
//!Generic instruction behavior method.
void ac_behavior( instruction ){

	int reg;

	// The PCE1 reg always points to the first instruction in the fetch packet
	// Notice that this will work if and only if the first instruction of the application is at address 0.
	if( ac_pc%8 == 0 )
		RB_C.write(PCE1, ac_pc);

	ac_pc +=4;

	//Testing Conditional Operations. See details at TMS320C6000 Manual, page 3-16.
	if( creg == 0 ){
		if( z != 0 )
			cerr << "Unknown Condition at pc: " << ac_pc - 4 << endl;
	}
	else {

		//Getting the register value.
		switch( creg )
			{
			case 1:  //Test register B0
				reg =  RB_B.read(0);
				break;

			case 2:  //Test register B1
				reg =  RB_B.read(1);
				break;

			case 3:  //Test register B2
				reg =  RB_B.read(2);
				break;

			case 4:  //Test register A1
				reg =  RB_A.read(1);
				break;

			case 5:  //Test register A2
				reg =  RB_A.read(2);
				break;

			default:
				cerr << "Unknown Condition register: " << creg << " at pc: " << ac_pc -4 <<endl;
			}
				
		//Testing according to z's  value.
		if (z){
			if( reg != 0 )
				ac_annul(); //Annuling instruction.
		}
		else{
			if( reg == 0 )
				ac_annul(); //Annuling instruction.
		}
	}	
		
	//Adjusting cycle count	
	if( p == 0 )
		cycle_count++;

	
};
 
//! Instruction Format behavior methods.
void ac_behavior( S_Oper ){
	

}

void ac_behavior( D_Oper ){

}

void ac_behavior( M_Oper ){

}

void ac_behavior( L_Oper ){

	//Getting the second operand. It may be cross loaded from the
	//register bank that is opposite to the destination reg. 
  // The s field indicates the destination register bank.
	if( x )
		if ( s ){
			xsrc1 = (int)RB_A.read(src1);
			xsrc2 = (int)RB_A.read(src2);
		}
		else{
			xsrc1 = (int)RB_B.read(src1);
			xsrc2 = (int)RB_B.read(src2);
		}
	else
		if ( s ){
			xsrc1 = (int)RB_B.read(src1);
			xsrc2 = (int)RB_B.read(src2);
		}
		else{
			xsrc1 = (int)RB_A.read(src1);
			xsrc2 = (int)RB_A.read(src2);
		}
}

void ac_behavior( SK_Oper ){

}

void ac_behavior( Branch ){
}

void ac_behavior( K_Oper ){
}

void ac_behavior( IDLE_Oper ){
}

void ac_behavior( NOP_Oper ){
}

//TODO: nonscaled constant offset
void ac_behavior( D_LDST_BaseR ){

	unsigned offset = 0;
	unsigned base = readReg(y, baseR);
	int addrmode;

	//Checking addressing mode
	addrmode = checkAMR(y, baseR);

	switch (mode){
	case 0x4:
		//Negative Offset with offsetR
		if(addrmode)
			offset = circular(scale_offset(ld_st, readReg(y, offsetR)));
		else
			offset = scale_offset(ld_st, readReg(y, offsetR));
		address = base - offset; 
		break;
	case 0x5:
		//Positive Offset with offsetR
		if(addrmode)
			offset = circular(scale_offset(ld_st, readReg(y, offsetR)));
		else
			offset = scale_offset(ld_st, readReg(y, offsetR));
		address = base + offset; 
		break;
	case 0xC:
		//Predecrement with offsetR
		if(addrmode)
			offset = circular(scale_offset(ld_st, readReg(y, offsetR)));
		else
			offset = scale_offset(ld_st, readReg(y, offsetR));
		address = --base + offset;
		writeReg(y, baseR, base);
		break;
	case 0xD:
		//Preincrement with offsetR
		if(addrmode)
			offset = circular(scale_offset(ld_st, readReg(y, offsetR)));
		else
			offset = scale_offset(ld_st, readReg(y, offsetR));
		address = ++base + offset;
		writeReg(y, baseR, base);
		break;
	case 0xE:
		//Posdecrement with offsetR
		if(addrmode)
			offset = circular(scale_offset(ld_st, readReg(y, offsetR)));
		else
			offset = scale_offset(ld_st, readReg(y, offsetR));
		address = base-- + offset;
		writeReg(y, baseR, base);
		break;
	case 0xF:
		//Posincrement with offsetR
		if(addrmode)
			offset = circular(scale_offset(ld_st, readReg(y, offsetR)));
		else
			offset = scale_offset(ld_st, readReg(y, offsetR));
		address = base++ + offset;
		writeReg(y, baseR, base);
		break;
	case 0x0:
		//Negative Offset with ucst5
		if(addrmode)
			offset = circular(scale_offset(ld_st, offsetR));
		else
			offset = scale_offset(ld_st, offsetR);

		address = base - offset; 
		break;
	case 0x1:
		//Positive Offset with ucst5
		offset = scale_offset(ld_st, offsetR);
		address = base + offset; 
		break;
	case 0x8:
		//Predecrement with ucst5
		if(addrmode)
			offset = circular(scale_offset(ld_st, offsetR));
		else
			offset = scale_offset(ld_st, offsetR);
		address = --base + offset;
		writeReg(y, baseR, base);
		break;
	case 0x9:
		//Preincrement with ucst5
		if(addrmode)
			offset = circular(scale_offset(ld_st, offsetR));
		else
			offset = scale_offset(ld_st, offsetR);
		address = ++base + offset;
		writeReg(y, baseR, base);
		break;
	case 0xA:
		//Posdecrement with ucst5
		if(addrmode)
			offset = circular(scale_offset(ld_st, offsetR));
		else
			offset = scale_offset(ld_st, offsetR);
		address = base-- + offset;
		writeReg(y, baseR, base);
		break;
	case 0xB:
		//Posincrement with ucst5
		if(addrmode)
			offset = circular(scale_offset(ld_st, offsetR));
		else
			offset = scale_offset(ld_st, offsetR);
		address = base++ + offset;
		writeReg(y, baseR, base);
		break;
	default:
		cerr << "Invalid mode for load/store address computation" <<endl;
		exit(1);
		break;
	}
}

void ac_behavior( D_LDST_K ){

	unsigned base;

	if( y == 0)
		base = 14;
	else
		base = 15;

	//Storing the first factor (base address) for address computation.
	address = readReg(1, base);
}
 
//!Instruction add_l_iii behavior method.
void ac_behavior( add_l_iii ){ 

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	writeReg(s, dst, ((int)readReg(s, src1) + xsrc2));


}
		


//!Instruction add_l_iil behavior method.
void ac_behavior( add_l_iil ){ 

	long long ldst;

  dprintf("%s r%d, r%d, r%d:r%d\n", get_name(), src1, src2, dst+1,dst);

	//Recording the 40-bit long result
	ldst =  (int) readReg(s, src1) + xsrc2;

  writeLong( s, dst, ldst );
}

//!Instruction add_l_ill behavior method.
void ac_behavior( add_l_ill ){ 

	long long lsrc2, ldst;

  dprintf("%s r%d, r%d:r%d, r%d:r%d\n", get_name(), src1, src2+1, src2, dst+1,dst);

	//Getting the 40-bit second operand
	lsrc2 = readLong( s, src2);

	//Storing result
	ldst = (int)xsrc1 + lsrc2;

	writeLong(s, dst, ldst);
	
}

//!Instruction add_l_cii behavior method.
void ac_behavior( add_l_cii ){ 

	sc_int<5> cst;

	//src1 is the 5-bit signed constant value
	cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);

	writeReg(s, dst, (int)cst + readReg(s, src2));
}

//!Instruction add_l_cll behavior method.
void ac_behavior( add_l_cll ){ 
	sc_int<5> cst;
	long long lsrc2, ldst;

	//src1 is the 5-bit signed constant value
	cst = src1;

  dprintf("%s %d, r%d:r%d, r%d:r%d\n", get_name(), (int)cst, src2, src2+1, dst, dst+1);

	//Getting the 40-bit second operand
	lsrc2 = readLong( s, src2);

	//Storing result
	ldst = cst + lsrc2;
	writeLong(s, dst, ldst);
}

//!Instruction addu_iil behavior method.
void ac_behavior( addu_iil ){ 
	long long ldst;

  dprintf("%s r%d, r%d, r%d:r%d\n", get_name(), src1, src2, dst+1,dst);

	//Recording the 40-bit long result
	ldst =  (unsigned int) readReg(s, src1) + (unsigned int)xsrc2;

  writeLong( s, dst, ldst );

}

//!Instruction addu_ill behavior method.
void ac_behavior( addu_ill ){ 
	long long lsrc2, ldst;

  dprintf("%s r%d, r%d:r%d, r%d:r%d\n", get_name(), src1, src2, src2+1, dst, dst+1);

	//Getting the 40-bit second operand
	lsrc2 = readLong( s, src2);

	//Recording the 40-bit long result
	ldst =  (unsigned int) xsrc1 + (unsigned int)lsrc2;

  writeLong( s, dst, ldst );
}

//!Instruction sub_l_iii behavior method.
void ac_behavior( sub_l_iii ){ 
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	writeReg(s, dst, ((int)readReg(s, src1) - xsrc2));
}

//!Instruction sub_l_xiii behavior method.
void ac_behavior( sub_l_xiii ){
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	writeReg(s, dst, (xsrc1 - (int)readReg(s, src2)));

}

//!Instruction sub_l_iil behavior method.
void ac_behavior( sub_l_iil ){
	long long ldst;

  dprintf("%s r%d, r%d, r%d:r%d\n", get_name(), src1, src2, dst+1,dst);

	//Recording the 40-bit long result
	ldst =  (int) readReg(s, src1) - xsrc2;

  writeLong( s, dst, ldst );

}

//!Instruction sub_l_xiil behavior method.
void ac_behavior( sub_l_xiil ){ 
	long long ldst;

  dprintf("%s r%d, r%d, r%d:r%d\n", get_name(), src1, src2, dst+1,dst);

	//Recording the 40-bit long result
	ldst =  xsrc1 - (int) readReg(s, src2);

  writeLong( s, dst, ldst );
}

//!Instruction sub_l_cii behavior method.
void ac_behavior( sub_l_cii ){ 
	sc_int<5> cst;

	//src1 is the 5-bit signed constant value
	cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);

	writeReg(s, dst, (int)cst - xsrc2);

}

//!Instruction sub_l_cll behavior method.
void ac_behavior( sub_l_cll ){
	sc_int<5> cst;
	long long lsrc2, ldst;


	//src1 is the 5-bit signed constant value
	cst = src1;
  dprintf("%s %d, r%d:r%d, r%d:r%d\n", get_name(), (int)cst, src2, src2+1, dst, dst+1);

	//Getting the 40-bit second operand
	lsrc2 = readLong( s, src2);

	//Storing result
	ldst = cst - lsrc2;
	writeLong(s, dst, ldst);
}

//!Instruction subu_iil behavior method.
void ac_behavior( subu_iil ){
	long long ldst;

  dprintf("%s r%d, r%d, r%d:r%d\n", get_name(), src1, src2, dst+1,dst);

	//Recording the 40-bit long result
	ldst =  (unsigned int) readReg(s, src1) - (unsigned int)xsrc2;

  writeLong( s, dst, ldst );
}

//!Instruction subu_xiil behavior method.
void ac_behavior( subu_xiil ){
	long long ldst;

  dprintf("%s r%d, r%d, r%d:r%d\n", get_name(), src1, src2, dst+1,dst);

	//Recording the 40-bit long result
	ldst =  (unsigned int)xsrc1 - (unsigned int) readReg(s, src2);

  writeLong( s, dst, ldst );
}

//!Instruction abs_ii behavior method.
void ac_behavior( abs_ii ){

  dprintf("%s r%d, r%d\n", get_name(), src2, dst);
	writeReg(s, dst, abs((int)xsrc2) );
	
}

//!Instruction abs_ll behavior method.
void ac_behavior( abs_ll ){
	long long ldst;

  dprintf("%s r%d:r%d, r%d:r%d\n", get_name(), src2+1, src2, dst+1,dst);


	//Recording the 40-bit long result
	ldst =  llabs( readLong(s, src2));

  writeLong( s, dst, ldst );
}

//!Instruction sadd_iii behavior method.
void ac_behavior( sadd_iii ){ 

	long long aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	aux = readReg(s, src1)+ xsrc2;

	//Saturation
	if( aux > INT_MAX ){
		writeReg(s, dst, INT_MAX);
		SATURATE;
	}
	else if (aux < INT_MIN ){
		writeReg(s, dst, INT_MIN);
		SATURATE;
	}
	else
		writeReg(s, dst, (int) aux);
		
}

//!Instruction sadd_ill behavior method.
void ac_behavior( sadd_ill ){ 

	long long aux;

  dprintf("%s r%d, r%d:r%d, r%d:r%d\n", get_name(), src1, src2+1, src2, dst+1,dst);

	aux = xsrc1 + readLong(s, src2);

	//Saturation
	if( aux > LONG_MAX ){
		writeLong(s, dst, LONG_MAX);
		SATURATE;
	}
	else if( aux < LONG_MIN ){
		writeLong(s, dst, LONG_MIN);
		SATURATE;
	}
	else
		writeLong(s, dst, aux);

}

//!Instruction sadd_cii behavior method.
void ac_behavior( sadd_cii ){ 
	long long aux;
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);

	aux = cst + xsrc2;

	//Saturation
	if( aux > INT_MAX ){
		writeReg(s, dst, INT_MAX);
		SATURATE;
	}
	else if (aux < INT_MIN ){
		writeReg(s, dst, INT_MIN);
		SATURATE;
	}
	else
		writeReg(s, dst, (int) aux);
		
}

//!Instruction sadd_cll behavior method.
void ac_behavior( sadd_cll ){ 


	long long aux;
	sc_int<5> cst;

  dprintf("%s r%d, r%d:r%d, r%d:r%d\n", get_name(), src1, src2+1, src2, dst+1,dst);

  cst = src1;

	aux = cst + readLong(s, src2);

	//Saturation
	if( aux > LONG_MAX ){
		writeLong(s, dst, LONG_MAX);
		SATURATE;
	}
	else if( aux < LONG_MIN ){
		writeLong(s, dst, LONG_MIN);
		SATURATE;
	}
	else
		writeLong(s, dst, aux);
}

//!Instruction ssub_iii behavior method.
void ac_behavior( ssub_iii ){ 

	long long aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	aux = readReg(s, src1)- xsrc2;

	//Saturation
	if( aux > INT_MAX ){
		writeReg(s, dst, INT_MAX);
		SATURATE;
	}
	else if (aux < INT_MIN ){
		writeReg(s, dst, INT_MIN);
		SATURATE;
	}
	else
		writeReg(s, dst, (int) aux);
		

}

//!Instruction ssub_xiii behavior method.
void ac_behavior( ssub_xiii ){ 

	long long aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	aux = xsrc1- readReg(s, src2);

	//Saturation
	if( aux > INT_MAX ){
		writeReg(s, dst, INT_MAX);
		SATURATE;
	}
	else if (aux < INT_MIN ){
		writeReg(s, dst, INT_MIN);
		SATURATE;
	}
	else
		writeReg(s, dst, (int) aux);
		
}

//!Instruction ssub_cii behavior method.
void ac_behavior( ssub_cii ){

	long long aux;
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);

	aux = cst - xsrc2;

	//Saturation
	if( aux > INT_MAX ){
		writeReg(s, dst, INT_MAX);
		SATURATE;
	}
	else if (aux < INT_MIN ){
		writeReg(s, dst, INT_MIN);
		SATURATE;
	}
	else
		writeReg(s, dst, (int) aux);
		
}

//!Instruction ssub_cll behavior method.
void ac_behavior( ssub_cll ){

	long long aux;
	sc_int<5> cst;

  dprintf("%s r%d, r%d:r%d, r%d:r%d\n", get_name(), src1, src2+1, src2, dst+1,dst);

  cst = src1;

	aux = cst - readLong(s, src2);

	//Saturation
	if( aux > LONG_MAX ){
		writeLong(s, dst, LONG_MAX);
		SATURATE;
	}
	else if( aux < LONG_MIN ){
		writeLong(s, dst, LONG_MIN);
		SATURATE;
	}
	else
		writeLong(s, dst, aux);
}

//!Instruction subc behavior method.
void ac_behavior( subc ){ 
	int aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	aux = readReg(s, src1) - xsrc2;
	
	if( aux >=0 ){
		aux <<=1;
		writeReg(s, dst, aux+1);
	}
	else {
		aux <<=1;
		writeReg(s, dst, aux);
	}
}

//!Instruction and_l_iii behavior method.
void ac_behavior( and_l_iii ){ 

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);
	writeReg( s, dst, readReg(s, src1) & xsrc2 );
}

//!Instruction and_l_cii behavior method.
void ac_behavior( and_l_cii ){
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);
	writeReg( s, dst, (int)cst & xsrc2 );
	
}

//!Instruction or_l_iii behavior method.
void ac_behavior( or_l_iii ){ 
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);
	writeReg( s, dst, readReg(s, src1) | xsrc2 );
}

//!Instruction or_l_cii behavior method.
void ac_behavior( or_l_cii ){
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);
	writeReg( s, dst, (int)cst | xsrc2 );
	
}

//!Instruction xor_l_iii behavior method.
void ac_behavior( xor_l_iii ){ 
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);
	writeReg( s, dst, readReg(s, src1) ^ xsrc2 );
}

//!Instruction xor_l_cii behavior method.
void ac_behavior( xor_l_cii ){
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);
	writeReg( s, dst, (int)cst ^ xsrc2 );
	
}

//!Instruction cmpeq_iii behavior method.
void ac_behavior( cmpeq_iii ){
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	if( readReg(s, src1) == xsrc2 )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
	
}

//!Instruction cmpeq_cii behavior method.
void ac_behavior( cmpeq_cii ){
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);

	if( (int)cst  == xsrc2 )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
	

}

//!Instruction cmpeq_ili behavior method.
void ac_behavior( cmpeq_ili ){

  dprintf("%s r%d, r%d:r%d, r%d\n", get_name(), src1, src2+1, src2, dst);

	if( xsrc1 == readLong(s, src2) )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
}

//!Instruction cmpeq_cli behavior method.
void ac_behavior( cmpeq_cli ){
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d:r%d, r%d\n", get_name(),  (int)cst, src2+1, src2, dst);

	if(  (int)cst == readLong(s, src2) )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
}

//!Instruction cmpgt_iii behavior method.
void ac_behavior( cmpgt_iii ){
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	if( readReg(s, src1) > xsrc2 )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
	
}

//!Instruction cmpgt_cii behavior method.
void ac_behavior( cmpgt_cii ){
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);

	if( (int)cst  > xsrc2 )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
	
}

//!Instruction cmpgt_ili behavior method.
void ac_behavior( cmpgt_ili ){
  dprintf("%s r%d, r%d:r%d, r%d\n", get_name(), src1, src2+1, src2, dst);

	if( xsrc1 > readLong(s, src2) )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
}

//!Instruction cmpgt_cli behavior method.
void ac_behavior( cmpgt_cli ){
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d:r%d, r%d\n", get_name(),  (int)cst, src2+1, src2, dst);

	if(  (int)cst > readLong(s, src2) )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
}

//!Instruction cmpgtu_iii behavior method.
void ac_behavior( cmpgtu_iii ){ 
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	if( (unsigned)readReg(s, src1) > (unsigned)xsrc2 )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
	
}

//!Instruction cmpgtu_cii behavior method.
void ac_behavior( cmpgtu_cii ){ 
	sc_int<5> cst;

  cst = src1;

  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);

	if( cst[4] ){
		cout << "MSB bit of the cst field is non-zero. Invalid result for this operation." << endl;
		return;
	}
	else{
		
		if( (unsigned int)cst.range(3,0)  > (unsigned)xsrc2 )
			writeReg(s, dst, 1);
		else
			writeReg(s, dst, 0);
	}
}

//!Instruction cmpgtu_ili behavior method.
void ac_behavior( cmpgtu_ili ){ 
  dprintf("%s r%d, r%d:r%d, r%d\n", get_name(), src1, src2+1, src2, dst);

	if( (unsigned)xsrc1 > (unsigned)readLong(s, src2) )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
}

//!Instruction cmpgtu_cli behavior method.
void ac_behavior( cmpgtu_cli ){

	sc_int<5> cst;

  cst = src1;

  dprintf("%s %d, r%d:r%d, r%d\n", get_name(), (int)cst, src2+1, src2,  dst);

	if( cst[4] ){
		cout << "MSB bit of the cst field is non-zero. Invalid result for this operation." << endl;
		return;
	}
	else{
		
		if( (unsigned int)cst.range(3,0)  > (unsigned long long)readLong(s, src2 ))
			writeReg(s, dst, 1);
		else
			writeReg(s, dst, 0);
	}
}

//!Instruction cmplt_iii behavior method.
void ac_behavior( cmplt_iii ){
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	if( readReg(s, src1) < xsrc2 )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
	
}

//!Instruction cmplt_cii behavior method.
void ac_behavior( cmplt_cii ){ 
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);

	if( (int)cst  < xsrc2 )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
	
}

//!Instruction cmplt_ili behavior method.
void ac_behavior( cmplt_ili ){
  dprintf("%s r%d, r%d:r%d, r%d\n", get_name(), src1, src2+1, src2, dst);

	if( xsrc1 < readLong(s, src2) )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
}

//!Instruction cmplt_cli behavior method.
void ac_behavior( cmplt_cli ){
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d:r%d, r%d\n", get_name(),  (int)cst, src2+1, src2, dst);

	if(  (int)cst < readLong(s, src2) )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
}

//!Instruction cmpltu_iii behavior method.
void ac_behavior( cmpltu_iii ){
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	if( (unsigned)readReg(s, src1) < (unsigned)xsrc2 )
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
}

//!Instruction cmpltu_cii behavior method.
void ac_behavior( cmpltu_cii ){
	sc_int<5> cst;

  cst = src1;

  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);

	if( cst[4] ){
		cout << "MSB bit of the cst field is non-zero. Invalid result for this operation." << endl;
		return;
	}
	else{
		
		if( (unsigned int)cst.range(3,0)  < (unsigned)xsrc2 )
			writeReg(s, dst, 1);
		else
			writeReg(s, dst, 0);
	}
}

//!Instruction cmpltu_ili behavior method.
void ac_behavior( cmpltu_ili ){
  dprintf("%s r%d, r%d:r%d, r%d\n", get_name(), src1, src2+1, src2, dst);

	if( (unsigned)xsrc1 < ((unsigned long long)readLong(s, src2) ))
		writeReg(s, dst, 1);
	else
		writeReg(s, dst, 0);
}

//!Instruction cmpltu_cli behavior method.
void ac_behavior( cmpltu_cli ){
	sc_int<5> cst;

  cst = src1;

  dprintf("%s %d, r%d:r%d, r%d\n", get_name(), (int)cst, src2+1, src2,  dst);

	if( cst[4] ){
		cout << "MSB bit of the cst field is non-zero. Invalid result for this operation." << endl;
		return;
	}
	else{
		
		if( (unsigned int)cst.range(3,0)  < (unsigned long long)readLong(s, src2 ))
			writeReg(s, dst, 1);
		else
			writeReg(s, dst, 0);
	}
}


//!Instruction lmbd_iii behavior method.
void ac_behavior( lmbd_iii ){ 

	int i;
	sc_int<32> data;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2,  dst);

	data = xsrc2;

	if( readReg(s, src1) % 2 ==0 ){
		//Search for the leftmost 0
		for(i = 31; i>=0; i--)
			if( data[i] == 0 )
				break;
	}
	else{
		//Search for the leftmost 1
		for(i = 31; i>=0; i--)
			if( data[i] == 1 )
				break;
	}

	if( i < 0 ){
		//Did not find
			writeReg(s, dst, 32);
	}
	else{
			writeReg(s, dst, 31 - i);
	}
		
}

//!Instruction lmbd_cii behavior method.
void ac_behavior( lmbd_cii ){

	int i;
	sc_int<32> data;

  dprintf("%s %d, r%d, r%d\n", get_name(), src1, src2, dst);


	data = xsrc2;

	//In this case src1 is a constant.
	if(  src1 % 2 ==0 ){
		//Search for the leftmost 0
		for(i = 31; i>=0; i--)
			if( data[i] == 0 )
				break;
	}
	else{
		//Search for the leftmost 1
		for(i = 31; i>=0; i--)
			if( data[i] == 1 )
				break;
	}

	if( i < 0 ){
		//Did not find
			writeReg(s, dst, 32);
	}
	else{
			writeReg(s, dst, 31 - i);
	}
		
}

//!Instruction norm_ii behavior method.
void ac_behavior( norm_ii ){ 

	int i;
	sc_int<32> data;

  dprintf("%s r%d, r%d\n", get_name(), src2, dst);

	data = xsrc2;
	
	if( data[31] == 0){
		//Search for the leftmost 1
		for(i = 30; i>=0; i--)
			if( data[i] == 1 )
				break;
	}
	else{
		//Search for the leftmost 0
		for(i = 30; i>=0; i--)
			if( data[i] == 0 )
				break;
	}

	//writing result
	if( i < 0 ){
		//Did not find
			writeReg(s, dst, 31);
	}
	else{
			writeReg(s, dst, 30 - i);
	}
	
}

//!Instruction norm_li behavior method.
void ac_behavior( norm_li ){
	int i;
	sc_int<40> data;

  dprintf("%s r%d:r%d, r%d\n", get_name(), src2+1, src2, dst);

	data = readLong(s, src2);
	
	if( data[39] == 0){
		//Search for the leftmost 1
		for(i = 38; i>=0; i--)
			if( data[i] == 1 )
				break;
	}
	else{
		//Search for the leftmost 0
		for(i = 38; i>=0; i--)
			if( data[i] == 0 )
				break;
	}

	//writing result
	if( i < 0 ){
		//Did not find
			writeReg(s, dst, 39);
	}
	else{
			writeReg(s, dst, 38 - i);
	}
	
}

//!Instruction sat behavior method.
void ac_behavior( sat ){ 

	long long aux;

  dprintf("%s r%d, r%d\n", get_name(), src1, src2, dst);

	aux = readLong(s, src2);

	//Saturation
	if( aux >= INT_MAX ){
		writeReg(s, dst, INT_MAX);
		SATURATE;
	}
	else if( aux < INT_MIN ){
		writeReg(s, dst, INT_MIN);
		SATURATE;
	}
	else
		writeReg(s, dst, aux);

}

//!Instruction mpy behavior method.
void ac_behavior( mpy ){ 

	short val1,val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 lsb of src1
	val1 = (short) (readReg(s,src1) & 0xFFFF);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (short) (xsrc2 & 0xFFFF);

	writeReg( s, dst, (int)(val1 * val2) );
}

//!Instruction mpy_k behavior method.
void ac_behavior( mpy_k ){ 
	short val1, val2;
	sc_int<5> cst;

  cst = src1;

  dprintf("%s %d, r%d, r%d\n", get_name(), cst.to_int(), src2, dst);

	//Getting the 16 lsb of src1
	val1 = (short) (cst);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (short) (xsrc2 & 0xFFFF);

	writeReg( s, dst, (int)(val1 * val2) );
}

//!Instruction mpyu behavior method.
void ac_behavior( mpyu ){ 
	unsigned short val1,val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 lsb of src1
	val1 = (unsigned short) (readReg(s,src1) & 0xFFFF);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (unsigned short)(xsrc2 & 0xFFFF);

	writeReg( s, dst, (unsigned int)(val1 * val2) );

}

//!Instruction mpyus behavior method.
void ac_behavior( mpyus ){ 
	unsigned short val1;
	short val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 lsb of src1
	val1 = (unsigned short) (readReg(s,src1) & 0xFFFF);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (short)(xsrc2 & 0xFFFF);

	writeReg( s, dst, (int)(val1 * val2) );

}

//!Instruction mpysu behavior method.
void ac_behavior( mpysu ){
	short val1;
	unsigned short val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 lsb of src1
	val1 = (short) (readReg(s,src1) & 0xFFFF);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (unsigned short)(xsrc2 & 0xFFFF);

	writeReg( s, dst, (int)(val1 * val2) );

}

//!Instruction mpysu_k behavior method.
void ac_behavior( mpysu_k ){
	short val1;
	unsigned short  val2;
	sc_int<5> cst;

  cst = src1;

  dprintf("%s %d, r%d, r%d\n", get_name(), cst.to_int(), src2, dst);

	//Getting the 16 lsb of src1
	val1 = (short) (cst);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (unsigned short) (xsrc2 & 0xFFFF);

	writeReg( s, dst, (int)(val1 * val2) );
}

//!Instruction mpyh behavior method.
void ac_behavior( mpyh ){

	short val1,val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 msb of src1
	val1 = (short) ((int)readReg(s,src1) >> 16);

	//Getting the 16 msb of src2, that may be cross loaded.
	val2 = (short) ((int)xsrc2 >> 16);

	writeReg( s, dst, (int)(val1 * val2) );
}


//!Instruction mpyhu behavior method.
void ac_behavior( mpyhu ){

	short val1,val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 msb of src1
	val1 = (unsigned short) ((unsigned int)readReg(s,src1) >> 16);

	//Getting the 16 msb of src2, that may be cross loaded.
	val2 = (unsigned short) ((unsigned int)xsrc2 >> 16);

	writeReg( s, dst, (unsigned int)(val1 * val2) );
}


//!Instruction mpyhus behavior method.
void ac_behavior( mpyhus ){

	unsigned short val1;
	short val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 msb of src1
	val1 = (unsigned short) ((unsigned int)readReg(s,src1) >> 16);

	//Getting the 16 msb of src2, that may be cross loaded.
	val2 = (short) ((int)xsrc2 >> 16);

	writeReg( s, dst, (int)(val1 * val2) );
}


//!Instruction mpyhsu behavior method.
void ac_behavior( mpyhsu ){

	short val1;
	unsigned short val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 msb of src1
	val1 = (short) ((int)readReg(s,src1) >> 16);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (unsigned short) ((unsigned int)xsrc2 >> 16);

	writeReg( s, dst, (int)(val1 * val2) );
}


//!Instruction mpyhl behavior method.
void ac_behavior( mpyhl ){
	short val1,val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 msb of src1
	val1 = (short) ((int)readReg(s,src1) >> 16);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (short) (xsrc2 & 0xFFFF);

	writeReg( s, dst, (int)(val1 * val2) );
}

//!Instruction mpyhlu behavior method.
void ac_behavior( mpyhlu ){
	unsigned short val1,val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 msb of src1
	val1 = (unsigned short) ((unsigned int)readReg(s,src1) >> 16);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (unsigned short) (xsrc2 & 0xFFFF);

	writeReg( s, dst, (unsigned int)(val1 * val2) );
}

//!Instruction mpyhuls behavior method.
void ac_behavior( mpyhuls ){
	unsigned short val1;
	short val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 msb of src1
	val1 = (unsigned short) ((unsigned int)readReg(s,src1) >> 16);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (short) (xsrc2 & 0xFFFF);

	writeReg( s, dst, (int)(val1 * val2) );
}

//!Instruction mpyhslu behavior method.
void ac_behavior( mpyhslu ){
	short val1;
	unsigned short val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 msb of src1
	val1 = (short) ((int)readReg(s,src1) >> 16);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (unsigned short) (xsrc2 & 0xFFFF);

	writeReg( s, dst, (int)(val1 * val2) );
}

//!Instruction mpylh behavior method.
void ac_behavior( mpylh ){
	short val1,val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 lsb of src1
	val1 = (short) (readReg(s,src1) & 0xFFFF );

	//Getting the 16 msb of src2, that may be cross loaded.
	val2 = (short) ((int)xsrc2 >> 16);

	writeReg( s, dst, (int)(val1 * val2) );
}

//!Instruction mpylhu behavior method.
void ac_behavior( mpylhu ){ 
	unsigned short val1,val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 lsb of src1
	val1 = (unsigned short) (readReg(s,src1) & 0xFFFF);

	//Getting the 16 msb of src2, that may be cross loaded.
	val2 = (unsigned short) ((unsigned int)xsrc2>> 16 );

	writeReg( s, dst, (unsigned int)(val1 * val2) );
}

//!Instruction mpyluhs behavior method.
void ac_behavior( mpyluhs ){
	unsigned short val1;
	short val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 msb of src1
	val1 = (unsigned short) (readReg(s,src1)& 0xFFFF);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (short) ((int)xsrc2  >> 16);

	writeReg( s, dst, (unsigned int)(val1 * val2) );
}

//!Instruction mpylshu behavior method.
void ac_behavior( mpylshu ){ 
	short val1;
	unsigned short val2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 msb of src1
	val1 = (short) (readReg(s,src1)& 0xFFFF);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (unsigned short) ((unsigned int)xsrc2  >> 16);

	writeReg( s, dst, (int)(val1 * val2) );
}

//!Instruction smpy behavior method.
void ac_behavior( smpy ){

	short val1,val2;
	int result;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 lsb of src1
	val1 = (short) (readReg(s,src1) & 0xFFFF);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (short) (xsrc2 & 0xFFFF);

	result = ((int)(val1 * val2))<<1;

	if( result != (int)0x80000000 )
		writeReg( s, dst, result );
	else{
		writeReg( s, dst, 0x7FFFFFFF );
		SATURATE;
	}

}

//!Instruction smpyh behavior method.
void ac_behavior( smpyh ){

	short val1,val2;
	int result;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 msb of src1
	val1 = (short) ((int)readReg(s,src1) >>16);

	//Getting the 16 msb of src2, that may be cross loaded.
	val2 = (short) ((int)xsrc2 >>16);

	result = ((int)(val1 * val2))<<1;

	if( result != (int)0x80000000 )
		writeReg( s, dst, result );
	else{
		writeReg( s, dst, 0x7FFFFFFF );
		SATURATE;
	}

}

//!Instruction smpyhl behavior method.
void ac_behavior( smpyhl ){

	short val1,val2;
	int result;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 msb of src1
	val1 = (short) ((int)readReg(s,src1) >>16);

	//Getting the 16 lsb of src2, that may be cross loaded.
	val2 = (short) (xsrc2 & 0xFFFF);

	result = ((int)(val1 * val2))<<1;

	if( result != (int)0x80000000 )
		writeReg( s, dst, result );
	else{
		writeReg( s, dst, 0x7FFFFFFF );
		SATURATE;
	}

}

//!Instruction smpylh behavior method.
void ac_behavior( smpylh ){

	short val1,val2;
	int result;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	//Getting the 16 lsb of src1
	val1 = (short) (readReg(s,src1)& 0xFFFF);

	//Getting the 16 msb of src2, that may be cross loaded.
	val2 = (short) ((int)xsrc2  >>16);

	result = ((int)(val1 * val2))<<1;

	if( result != (int)0x80000000 )
		writeReg( s, dst, result );
	else{
		writeReg( s, dst, 0x7FFFFFFF );
		SATURATE;
	}

}

//!Instruction add_d_iii behavior method.
void ac_behavior( add_d_iii ){
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	writeReg(s, dst, ((int)readReg(s, src1) + (int)readReg(s,src2)));
}


//!Instruction add_ici behavior method.
void ac_behavior( add_ici ){
	sc_uint<5> cst;

	//src2 is the 5-bit signed constant value
	cst = src2;
  dprintf("%s %d, r%d, r%d\n", get_name(), (unsigned int)cst, src2, dst);

	writeReg(s, dst, (int)readReg(s, src1) + (unsigned int)cst);

}

//!Instruction addab_iii behavior method.
void ac_behavior( addab_iii ){ 

	int mode;
	int operand1 = (int)readReg(s, src1);
	
  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1,  dst);

	mode = checkAMR(s, src2);

	if( mode ){
		circular(operand1);
	}

	//Doing the addition
	writeReg(s, dst, operand1 + (int)readReg(s, src2));
	dprintf("Result: %x\n",operand1 + (int)readReg(s, src2));
 }

//!Instruction addah_iii behavior method.
void ac_behavior( addah_iii ){ 

	int mode;
	int operand1 = (int)readReg(s, src1);
	
  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1,  dst);

	mode = checkAMR(s, src2);

	operand1<<=1;

	if( mode ){
		circular(operand1);
	}


	//Doing the addition
	writeReg(s, dst, operand1 + (int)readReg(s, src2));
	dprintf("Result: %x\n",operand1 + (int)readReg(s, src2));

}

//!Instruction addaw_iii behavior method.
void ac_behavior( addaw_iii ){ 
	int mode;
	int operand1 = (int)readReg(s, src1);
	
  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1,  dst);

	mode = checkAMR(s, src2);

	operand1<<=2;

	if( mode ){
		circular(operand1);
	}


	//Doing the addition
	writeReg(s, dst, operand1 + (int)readReg(s, src2));
	dprintf("Result: %x\n",operand1 + (int)readReg(s, src2));

}

//!Instruction addab_ici behavior method.
void ac_behavior( addab_ici ){ 
	int mode;
	unsigned operand1 = (unsigned)src1;
	
  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1,  dst);

	mode = checkAMR(s, src2);

	if( mode ){
		circular(operand1);
	}


	//Doing the addition
	writeReg(s, dst, operand1 + (int)readReg(s, src2));
	dprintf("Result: %x\n",operand1 + (int)readReg(s, src2));
}

//!Instruction addah_ici behavior method.
void ac_behavior( addah_ici ){
	int mode;
	unsigned operand1 = (unsigned)src1;
	
  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1,  dst);

	mode = checkAMR(s, src2);

	operand1<<=1;

	if( mode ){
		circular(operand1);
	}


	//Doing the addition
	writeReg(s, dst, operand1 + (int)readReg(s, src2));
	dprintf("Result: %x\n",operand1 + (int)readReg(s, src2));
}

//!Instruction addaw_ici behavior method.
void ac_behavior( addaw_ici ){
	int mode;
	unsigned operand1 = (unsigned)src1;
	
  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1,  dst);

	mode = checkAMR(s, src2);

	operand1<<=2;

	if( mode ){
		circular(operand1);
	}


	//Doing the addition
	writeReg(s, dst, operand1 + (int)readReg(s, src2));
	dprintf("Result: %x\n",operand1 + (int)readReg(s, src2));
}

//!Instruction sub_d_iii behavior method.
void ac_behavior( sub_d_iii ){
  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);

	writeReg(s, dst, ((int)readReg(s,src2)- (int)readReg(s, src1)));
}

//!Instruction sub_d_ici behavior method.
void ac_behavior( sub_d_ici ){ 
	sc_uint<5> cst;

	//src1 is the 5-bit unsigned constant value
	cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), src2, (unsigned int)cst, dst);

	writeReg(s, dst, (int)readReg(s, src2) - (unsigned int)cst );

}

//!Instruction subab_iii behavior method.
void ac_behavior( subab_iii ){ 

	int mode;
	int operand1 = (int)readReg(s, src1);
	
  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1,  dst);

	mode = checkAMR(s, src2);

	if( mode ){
		circular(operand1);
	}


	//Doing the addition
	writeReg(s, dst,  (int)readReg(s, src2) - operand1 );
	dprintf("Result: %x\n",(int)readReg(s, src2) - operand1);
}

//!Instruction subab_ici behavior method.
void ac_behavior( subab_ici ){
	int mode;
	unsigned operand1 = (unsigned) src1;
	
  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1,  dst);

	mode = checkAMR(s, src2);

	if( mode ){
		circular(operand1);
	}


	//Doing the addition
	writeReg(s, dst,  (int)readReg(s, src2) - operand1 );
	dprintf("Result: %x\n",(int)readReg(s, src2) - operand1);
}

//!Instruction subah_iii behavior method.
void ac_behavior( subah_iii ){

	int mode;
	int operand1 = (int)readReg(s, src1);
	
  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1,  dst);

	mode = checkAMR(s, src2);

	operand1 <<=1;

	if( mode ){
		circular(operand1);
	}


	//Doing the addition
	writeReg(s, dst,  (int)readReg(s, src2) - operand1 );
	dprintf("Result: %x\n",(int)readReg(s, src2) - operand1);
}

//!Instruction subah_ici behavior method.
void ac_behavior( subah_ici ){ 
	int mode;
	unsigned operand1 = (unsigned) src1;
	
  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1,  dst);

	mode = checkAMR(s, src2);

	operand1 <<=1;

	if( mode ){
		circular(operand1);
	}


	//Doing the addition
	writeReg(s, dst,  (int)readReg(s, src2) - operand1 );
	dprintf("Result: %x\n",(int)readReg(s, src2) - operand1);
}

//!Instruction subaw_iii behavior method.
void ac_behavior( subaw_iii ){ 

	int mode;
	int operand1 = (int)readReg(s, src1);
	
  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1,  dst);

	mode = checkAMR(s, src2);

	operand1 <<=2;

	if( mode ){
		circular(operand1);
	}


	//Doing the addition
	writeReg(s, dst,  (int)readReg(s, src2) - operand1 );
	dprintf("Result: %x\n",(int)readReg(s, src2) - operand1);
}

//!Instruction subaw_ici behavior method.
void ac_behavior( subaw_ici ){

	int mode;
	unsigned operand1 = (unsigned) src1;
	
  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1,  dst);

	mode = checkAMR(s, src2);

	operand1 <<=2;

	if( mode ){
		circular(operand1);
	}

	//Doing the addition
	writeReg(s, dst,  (int)readReg(s, src2) - operand1 );
	dprintf("Result: %x\n",(int)readReg(s, src2) - operand1);
}

//!Instruction add_s_iii behavior method.
void ac_behavior( add_s_iii ){ 

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	writeReg(s, dst, ((int)readReg(s, src1) + (int)xsrc2));

}

//!Instruction add_s_cii behavior method.
void ac_behavior( add_s_cii ){ 

	sc_int<5> cst;

	//src1 is the 5-bit signed constant value
	cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);

	writeReg(s, dst, (int)cst + readReg(s, src2));
}

//!Instruction add2 behavior method.
void ac_behavior( add2 ){

	int lsbsrc1, lsbsrc2, msbsrc1, msbsrc2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	lsbsrc1 = 0xFFFF & ((int)readReg(s, src1));
	lsbsrc2 = 0xFFFF & ((int)xsrc2);
	msbsrc1 = ((int)readReg(s, src1)) >> 16;
	msbsrc2 = ((int)xsrc2) >> 16;

	writeReg(s, dst, ((msbsrc1+msbsrc2) <<16) | ((lsbsrc1+lsbsrc2)&0xFFFF));
	dprintf("Result: %d\n", ((msbsrc1+msbsrc2) <<16) | ((lsbsrc1+lsbsrc2)&0xFFFF));

}

//!Instruction clr behavior method.
void ac_behavior( clr ){ 

	int i;
	sc_uint<5> lsb, msb;
	sc_uint<32> field, src2a;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);
	field = readReg(s, src1);
	src2a = xsrc2;

	lsb = field.range(9,5);
	msb = field.range(4,0);

	for( i = (int) lsb; i<= (int)msb; i++)
		src2a[i] = 0;

	writeReg(s, dst, (int)src2a);
	dprintf("Result: %d\n", (int)src2a);
	
}


//!Instruction ext behavior method.
void ac_behavior( ext ){ 

	sc_uint<5> left, right;
	sc_uint<32> field;
	int src2a;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);
	field = readReg(s, src1);
	src2a = xsrc2;

	right = field.range(4,0);
	left = field.range(9,5);

	src2a <<= left;
	(int)src2a >> right; 

	writeReg(s, dst, (int)src2a);
	dprintf("Result: %d\n", (int)src2a);
	
}

//!Instruction extu behavior method.
void ac_behavior( extu ){

	sc_uint<5> left, right;
	sc_uint<32> field;
	unsigned int src2a;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);
	field = readReg(s, src1);
	src2a = xsrc2;

	right = field.range(4,0);
	left = field.range(9,5);

	src2a <<= left;
	(unsigned int)src2a >> right; 

	writeReg(s, dst, (unsigned int)src2a);
	dprintf("Result: %d\n", (unsigned)src2a);
	
}

//!Instruction and_s_iii behavior method.
void ac_behavior( and_s_iii ){
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);
	writeReg( s, dst, readReg(s, src1) & xsrc2 );
}

//!Instruction and_s_cii behavior method.
void ac_behavior( and_s_cii ){ 
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);
	writeReg( s, dst, (int)cst & xsrc2 );
	
}

//!Instruction or_s_iii behavior method.
void ac_behavior( or_s_iii ){ 
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);
	writeReg( s, dst, readReg(s, src1) | xsrc2 );
}

//!Instruction or_s_cii behavior method.
void ac_behavior( or_s_cii ){
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);
	writeReg( s, dst, (int)cst | xsrc2 );
	
}

//!Instruction xor_s_iii behavior method.
void ac_behavior( xor_s_iii ){
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);
	writeReg( s, dst, readReg(s, src1) ^ xsrc2 );
}

//!Instruction xor_s_cii behavior method.
void ac_behavior( xor_s_cii ){
	sc_int<5> cst;

  cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), (int)cst, src2, dst);
	writeReg( s, dst, (int)cst ^ xsrc2 );
	
}

//!Instruction mvc_cr behavior method.
void ac_behavior( mvc_cr ){ 

  dprintf("%s r%d, r%d\n", get_name(), src2, dst);
	writeReg(s, dst, RB_C.read(src2));
	
}

//!Instruction mvc_rc behavior method.
void ac_behavior( mvc_rc ){ 
	
  dprintf("%s r%d, r%d\n", get_name(), src2, dst);
	RB_C.write( dst, xsrc2 );
}

//!Instruction set behavior method.
void ac_behavior( set_field ){

	int i;
	sc_uint<5> lsb, msb;
	sc_uint<32> field, src2a;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);
	field = readReg(s, src1);
	src2a = xsrc2;

	lsb = field.range(9,5);
	msb = field.range(4,0);

	for( i = (int) lsb; i<= (int)msb; i++)
		src2a[i] = 1;

	writeReg(s, dst, (int)src2a);
	dprintf("Result: %d\n", (int)src2a);
}

//!Instruction sshl_iii behavior method.
void ac_behavior( sshl_iii ){ 
	unsigned shift;
	sc_uint<32> aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);

	//Five LSB are the shift amount
	shift = readReg(s, src1)&0x1F;


	//Saturation
	aux = xsrc2;
	if( ( aux.range(31, 31-shift) == 0) || ( aux.range(31, 31-shift).and_reduce())){
		writeReg(s, dst, (xsrc2 << shift));
		dprintf("Result: %d\n", (xsrc2 << shift) );
	}
	else if( (int) xsrc2>0 ){
		writeReg(s, dst, INT_MAX);
		SATURATE;
		dprintf("Result: %x\n",  INT_MAX);
	}
	else if ((int)xsrc2 < 0 ){
		writeReg(s, dst, INT_MIN);
		SATURATE;
		dprintf("Result: %x\n",  INT_MIN);
	}
}

//!Instruction sshl_ici behavior method.
void ac_behavior( sshl_ici ){ 
	unsigned shift;
	sc_uint<32> aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);

	//In this case, src1 is ucst5.
	shift = src1&0x1F;

	//Saturation
	aux = xsrc2;
	if( ( aux.range(31, 31-shift) == 0) || ( aux.range(31, 31-shift).and_reduce())){
		writeReg(s, dst, (xsrc2 << shift));
		dprintf("Result: %d\n", (xsrc2 << shift) );
	}
	else if( (int) xsrc2>0 ){
		writeReg(s, dst, INT_MAX);
		SATURATE;
		dprintf("Result: %x\n",  INT_MAX);
	}
	else if ((int)xsrc2 < 0 ){
		writeReg(s, dst, INT_MIN);
		SATURATE;
		dprintf("Result: %x\n",  INT_MIN);
	}
}

//!Instruction shl_iii behavior method.
void ac_behavior( shl_iii ){ 
	unsigned shift;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);
	//Six LSB are the shift amount
	shift = readReg(s, src1)&0x3F;

	if(shift > 39)
		shift = 40;

	writeReg(s, dst, (xsrc2 << shift));
	dprintf("Result: %d\n",  (xsrc2 << shift));

}

//!Instruction shl_lil behavior method.
void ac_behavior( shl_lil ){
	unsigned shift;
	long long aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);

	aux = readLong(s, src2);
	shift = readReg(s, src1)&0x3F;

	if(shift > 39)
		shift = 40;

	writeLong(s, dst, (aux << shift));
	dprintf("Result: %d\n",  (aux << shift));
}

//!Instruction shl_iil behavior method.
void ac_behavior( shl_iil ){
	unsigned shift;
	long long aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);

	aux = (long long)xsrc2;
	shift = readReg(s, src1)&0x3F;

	if(shift > 39)
		shift = 40;

	writeLong(s, dst, (aux << shift));
	dprintf("Result: %d\n",  (aux << shift));
}

//!Instruction shl_ici behavior method.
void ac_behavior( shl_ici ){
	unsigned shift;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);
	shift = src1&0x3F;

	if(shift > 39)
		shift = 40;

	writeReg(s, dst, (xsrc2 << shift));
	dprintf("Result: %d\n",  (xsrc2 << shift));

}

//!Instruction shl_lcl behavior method.
void ac_behavior( shl_lcl ){
	unsigned shift;
	long long aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);

	aux = readLong(s, src2);
	shift = src1&0x3F;

	if(shift > 39)
		shift = 40;

	writeLong(s, dst, (aux << shift));
	dprintf("Result: %d\n",  (aux << shift));
}

//!Instruction shl_icl behavior method.
void ac_behavior( shl_icl ){
	unsigned shift;
	long long aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);

	aux = (long long)xsrc2;
	shift = src1&0x3F;

	if(shift > 39)
		shift = 40;

	writeLong(s, dst, (aux << shift));
	dprintf("Result: %d\n",  (aux << shift));
}

//!Instruction shr_iii behavior method.
void ac_behavior( shr_iii ){
	unsigned shift;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);
	shift = readReg(s, src1)&0x3F;

	if(shift > 39)
		shift = 40;

	writeReg(s, dst, ((int)xsrc2 >> shift));
	dprintf("Result: %d\n",  ((int)xsrc2 >> shift));

}

//!Instruction shr_lil behavior method.
void ac_behavior( shr_lil ){
	unsigned shift;
	long long aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);

	aux = readLong(s, src2);
	shift = readReg(s, src1)&0x3F;

	if(shift > 39)
		shift = 40;

	writeLong(s, dst, (aux >> shift));
	dprintf("Result: %d\n",  (aux >> shift));
}

//!Instruction shr_ici behavior method.
void ac_behavior( shr_ici ){
	unsigned shift;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);
	shift = src1&0x3F;

	if(shift > 39)
		shift = 40;

	writeReg(s, dst, (xsrc2 << shift));
	dprintf("Result: %d\n",  (xsrc2 << shift));

}

//!Instruction shr_lcl behavior method.
void ac_behavior( shr_lcl ){
	unsigned shift;
	long long aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);

	aux = readLong(s, src2);
	shift = src1&0x3F;

	if(shift > 39)
		shift = 40;

	writeLong(s, dst, (aux >> shift));
	dprintf("Result: %d\n",  (aux >> shift));
}

//!Instruction shru_iii behavior method.
void ac_behavior( shru_iii ){
	unsigned shift;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);
	shift = readReg(s, src1)&0x3F;

	if(shift > 39)
		shift = 40;

	writeReg(s, dst, ((unsigned)xsrc2 >> shift));
	dprintf("Result: %d\n",  ((unsigned)xsrc2 >> shift));

}

//!Instruction shru_lil behavior method.
void ac_behavior( shru_lil ){
	unsigned shift;
	unsigned long long aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);

	aux = readLong(s, src2);
	shift = readReg(s, src1)&0x3F;

	if(shift > 39)
		shift = 40;

	writeLong(s, dst, (aux >> shift));
	dprintf("Result: %d\n",  (aux >> shift));
}

//!Instruction shru_ici behavior method.
void ac_behavior( shru_ici ){
	unsigned shift;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);
	shift = src1&0x3F;

	if(shift > 39)
		shift = 40;

	writeReg(s, dst, ((unsigned)xsrc2 >> shift));
	dprintf("Result: %d\n",  ((unsigned)xsrc2 >> shift));

}

//!Instruction shru_lcl behavior method.
void ac_behavior( shru_lcl ){
	unsigned shift;
	unsigned long long aux;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src2, src1, dst);

	aux = (unsigned long long) readLong(s, src2);
	shift = src1&0x3F;

	if(shift > 39)
		shift = 40;

	writeLong(s, dst, (aux >> shift));
	dprintf("Result: %d\n",  (aux >> shift));
}

//!Instruction sub_s_iii behavior method.
void ac_behavior( sub_s_iii ){ 
  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	writeReg(s, dst, ((int)readReg(s, src1) - (int)xsrc2));
}

//!Instruction sub_s_cii behavior method.
void ac_behavior( sub_s_cii ){
	sc_int<5> cst;

	//src1 is the 5-bit signed constant value
	cst = src1;
  dprintf("%s %d, r%d, r%d\n", get_name(), src2,(int)cst, dst);

	writeReg(s, dst, (int)cst -(int)xsrc2);

}

//!Instruction sub2 behavior method.
void ac_behavior( sub2 ){
	int lsbsrc1, lsbsrc2, msbsrc1, msbsrc2;

  dprintf("%s r%d, r%d, r%d\n", get_name(), src1, src2, dst);

	lsbsrc1 = 0xFFFF & ((int)readReg(s, src1));
	lsbsrc2 = 0xFFFF & ((int)xsrc2);
	msbsrc1 = ((int)readReg(s, src1)) >> 16;
	msbsrc2 = ((int)xsrc2) >> 16;

	writeReg(s, dst, ((msbsrc1-msbsrc2) <<16) | ((lsbsrc1-lsbsrc2)&0xFFFF));
	dprintf("Result: %d\n", ((msbsrc1-msbsrc2) <<16) | ((lsbsrc1-lsbsrc2)&0xFFFF));

}

//!Instruction ext_k behavior method.
void ac_behavior( ext_k ){ 

  dprintf("%s r%d, %d, %d, r%d\n", get_name(), src2, csta, cstb, dst);
	int src2a;

	src2a = xsrc2;

	src2a <<= csta;
	src2a = (int)src2a >> cstb; 

	writeReg(s, dst, (int)src2a);
	dprintf("Result: %d\n", (int)src2a);
}

//!Instruction extu_k behavior method.
void ac_behavior( extu_k ){ 

	unsigned int src;
  dprintf("%s r%d, %d, %d, r%d\n", get_name(), src2, csta, cstb, dst);

	src = readReg(s, src2);

	src <<= csta;
	(int)src >>= cstb; 

	writeReg(s, dst, (int)src);
	dprintf("Result: %d\n", (int)src);

}

//!Instruction clr_k behavior method.
void ac_behavior( clr_k ){

	int i;
	sc_uint<32> src;

  dprintf("%s r%d, %d, %d, r%d\n", get_name(), src2, csta, cstb, dst);
	src = readReg(s, src2);

	for( i = csta; i<= (int)cstb; i++)
		src[i] = 0;

	writeReg(s, dst, (int)src);
	dprintf("Result: %d\n", (int)src);
}

//!Instruction set_k behavior method.
void ac_behavior( set_k ){

	int i;
	sc_uint<32> src;

  dprintf("%s r%d, %d, %d, r%d\n", get_name(), src2, csta, cstb, dst);
	src = readReg(s, src2);

	for( i = csta; i<= (int)cstb; i++)
		src[i] = 1;

	writeReg(s, dst, (int)src);
	dprintf("Result: %d\n", (int)src);
}

//!Instruction addk behavior method.
void ac_behavior( addk ){

  dprintf("%s %d, r%d\n", get_name(), cst, dst);

	writeReg(s, dst, ( cst + readReg(s,dst)));
	dprintf("Result: %d\n", ( cst + readReg(s,dst)));
}

//!Instruction mvk behavior method.
void ac_behavior( mvk ){

  dprintf("%s %d, r%d\n", get_name(), cst, dst);

	writeReg(s, dst, cst);
	dprintf("Result: %d\n",  cst );
}

//!Instruction mvkh behavior method.
void ac_behavior( mvkh ){

	int lsb;
  dprintf("%s %d, r%d\n", get_name(), cst, dst);

	lsb = readReg(s, dst) & 0xFFFF;

	writeReg(s, dst, (cst<<16)| lsb);
	dprintf("Result: %d\n",(cst<<16)| lsb  );
}

//!Instruction b_lab behavior method.
void ac_behavior( b_lab ){
	
	int target;

  dprintf("%s %d\n", get_name(), cst_b);

	//TODO: Conferir com cuidado PCE1
	target = (cst_b<<2) + RB_C.read(PCE1);

	//TODO: Isso soh funcionara direito se existir um controle sobre o paralelismo,
	//      isto e, instrucoes q seriam executadas em paralelo ocupam o mesmo delay slot.
	//      Ver exemplo no manual do TMSC6000 pagina 3-41.
	ac_pc = delay(target, 5);

	dprintf("Result: %d\n",target  );
}

//!Instruction b_reg behavior method.
void ac_behavior( b_reg ){

  dprintf("%s %d\n", get_name(), src2);

	//TODO: Isso soh funcionara direito se existir um controle sobre o paralelismo,
	//      isto e, instrucoes q seriam executadas em paralelo ocupam o mesmo delay slot.
	//      Ver exemplo no manual do TMSC6000 pagina 3-41.
	ac_pc = delay(readReg(s, src2), 5);

	dprintf("Result: %d\n", readReg(s, src2)  );
}


//!Instruction b_irp behavior method.
void ac_behavior( b_irp ){

  dprintf("%s %d\n", get_name(), src2);

	//TODO: Isso soh funcionara direito se existir um controle sobre o paralelismo,
	//      isto e, instrucoes q seriam executadas em paralelo ocupam o mesmo delay slot.
	//      Ver exemplo no manual do TMSC6000 pagina 3-41.
	ac_pc = delay(RB_C.read(IRP), 5);

	//TODO: Copy PGIE to GIE if an interrupt control system should be added in the future.
	//      See TMSC6000 ISA manual page 3-44.

	dprintf("Result: %d\n", RB_C.read(IRP));
}


//!Instruction b_nrp behavior method.
void ac_behavior( b_nrp ){ 

  dprintf("%s %d\n", get_name(), src2);

	//TODO: Isso soh funcionara direito se existir um controle sobre o paralelismo,
	//      isto e, instrucoes q seriam executadas em paralelo ocupam o mesmo delay slot.
	//      Ver exemplo no manual do TMSC6000 pagina 3-41.
	ac_pc = delay( RB_C.read(NRP), 5);

	//TODO: Set NMIE if an interrupt control system should be added in the future.
	//      See TMSC6000 ISA manual page 3-46.

	dprintf("Result: %d\n", RB_C.read(NRP));

}

//!Instruction idle behavior method.
void ac_behavior( idle ){ 
	dprintf("IDLE does nothing.");
}

//!Instruction nop behavior method.
void ac_behavior( nop ){ 

	//TODO:Is it necessary to have a multi-cycle NOP in this functional model?
  dprintf("%s %d\n", get_name(), src);

}

//!Instruction ldb behavior method.
void ac_behavior( ldb ){ 

  dprintf("%s *+r%d [%d], r%d \tmode: %x\n", get_name(), baseR, offsetR, dst, mode);

	writeReg(s, dst, (int)MEM.read_byte(address));
	dprintf("Result: %d\n", (int)MEM.read_byte(address));
}

//!Instruction ldbu behavior method.
void ac_behavior( ldbu ){

  dprintf("%s *+r%d [%d], r%d \tmode: %x\n", get_name(), baseR, offsetR, dst, mode);

	writeReg(s, dst, (unsigned int)MEM.read_byte(address));
	dprintf("Result: %d\n", (unsigned int)MEM.read_byte(address));
}

//!Instruction ldh behavior method.
void ac_behavior( ldh ){

  dprintf("%s *+r%d [%d], r%d \tmode: %x\n", get_name(), baseR, offsetR, dst, mode);

	writeReg(s, dst, (int)MEM.read_half(address));
	dprintf("Result: %d\n", (int)MEM.read_half(address));
}

//!Instruction ldhu behavior method.
void ac_behavior( ldhu ){ 

  dprintf("%s *+r%d [%d], r%d \tmode: %x\n", get_name(), baseR, offsetR, dst, mode);

	writeReg(s, dst, (unsigned int)MEM.read_half(address));
	dprintf("Result: %d\n", (unsigned int)MEM.read_half(address));
}


//!Instruction ldw behavior method.
void ac_behavior( ldw ){

  dprintf("%s *+r%d [%d], r%d \tmode: %x\n", get_name(), baseR, offsetR, dst, mode);

	writeReg(s, dst, (int)MEM.read(address));
	dprintf("Result: %d\n", (int)MEM.read(address));
}

//!Instruction ldb_k behavior method.
void ac_behavior( ldb_k ){ 

  dprintf("%s *+r%d [%d], r%d\n", get_name(), y+14, ucst, dst);

	writeReg(s, dst, (int)MEM.read_byte(address + ucst));
	dprintf("Result: %d\n", (int)MEM.read_byte(address + ucst));
}

//!Instruction ldbu_k behavior method.
void ac_behavior( ldbu_k ){

  dprintf("%s *+r%d [%d], r%d\n", get_name(), y+14, ucst, dst);

	writeReg(s, dst, (unsigned int)MEM.read_byte(address + ucst));
	dprintf("Result: %d\n", (unsigned int)MEM.read_byte(address + ucst));
}

//!Instruction ldh_k behavior method.
void ac_behavior( ldh_k ){

  dprintf("%s *+r%d [%d], r%d\n", get_name(), y+14, ucst, dst);

	//scaling offset
	ucst <<= 1;

	writeReg(s, dst, (int)MEM.read_half(address + ucst));
	dprintf("Result: %d\n", (int)MEM.read_half(address + ucst));
}

//!Instruction ldhu_k behavior method.
void ac_behavior( ldhu_k ){

  dprintf("%s *+r%d [%d], r%d\n", get_name(), y+14, ucst, dst);

	//scaling offset
	ucst <<= 1;

	writeReg(s, dst, (unsigned int)MEM.read_half(address + ucst));
	dprintf("Result: %d\n", (unsigned int)MEM.read_half(address + ucst));
}

//!Instruction ldw_k behavior method.
void ac_behavior( ldw_k ){

  dprintf("%s *+r%d [%d], r%d\n", get_name(), y+14, ucst, dst);

	//scaling offset
	ucst <<= 2;

	writeReg(s, dst, MEM.read(address + ucst));
	dprintf("Result: %d\n", MEM.read(address + ucst));
}

//!Instruction stb behavior method.
void ac_behavior( stb ){

	char src=0;

  dprintf("%s r%d, *+r%d [%d]\n", get_name(), baseR, offsetR, dst, mode);

	//Getting the data to be stored.
	src = readReg(s, dst) & 0xFF;

	MEM.write_byte(address, src);
	dprintf("Result: %d\n", src);
}

//!Instruction sth behavior method.
void ac_behavior( sth ){

	short src=0;

  dprintf("%s r%d, *+r%d [%d]\n", get_name(), baseR, offsetR, dst, mode);

	//Getting the data to be stored.
	src = readReg(s, dst) & 0xFFFF;

	MEM.write_half(address, src);
	dprintf("Result: %d\n", src);
}

//!Instruction stw behavior method.
void ac_behavior( stw ){

	int src=0;

  dprintf("%s r%d, *+r%d [%d]\n", get_name(), baseR, offsetR, dst, mode);

	//Getting the data to be stored.
	src = readReg(s, dst);

	MEM.write(address, src);
	dprintf("Result: %d\n", src);
}

//!Instruction stb_k behavior method.
void ac_behavior( stb_k ){
	char src=0;
 
	dprintf("%s r%d, *+r%d [%d]\n", get_name(), dst, y+14, ucst);

	//Getting the data to be stored.
	src = readReg(s, dst) & 0xFF;

	MEM.write_byte(address+ucst, src);
	dprintf("Result: %d\n", src);
}

//!Instruction sth_k behavior method.
void ac_behavior( sth_k ){ 
	short src=0;
 
	dprintf("%s r%d, *+r%d [%d]\n", get_name(), dst, y+14, ucst);

	//scaling offset
	ucst <<= 1;

	//Getting the data to be stored.
	src = readReg(s, dst) & 0xFFFF;

	MEM.write_half(address+ucst, src);
	dprintf("Result: %d\n", src);
}

//!Instruction stw_k behavior method.
void ac_behavior( stw_k ){
 	int src=0;

	dprintf("%s r%d, *+r%d [%d]\n", get_name(), dst, y+14, ucst);

	//scaling offset
	ucst <<= 2;

	//Getting the data to be stored.
	src = readReg(s, dst);

	MEM.write(address+ucst, src);
	dprintf("Result: %d\n", src);
}

