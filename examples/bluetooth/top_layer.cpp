// sysc1.cpp : It defines the input point of the application control.
//
#include <systemc.h>
#include "RF_modules.h"
#include "DSP_modules"
#include "modulator"
//#include "modules_test"

class bit_generator : public sc_module
{
public:
	sc_in  <bool> clock2;
	sc_out <bool> uscita;

	SC_HAS_PROCESS(bit_generator);
	bit_generator (sc_module_name name);
private:
	void rand_bit ();
	const char *sequence;
	int index;
};


bit_generator::bit_generator (sc_module_name name) : sc_module(name)
{
	SC_METHOD(rand_bit);
	sensitive << clock2.pos();
	index = 0;
	sequence =
#include "randomseq.txt"
	; // arrayx definition
	// bit stream used in paper: "00111000000101010111110010110010110011100101000110"
}

void bit_generator::rand_bit ()
{
	bool value = sequence[index] == '1';
	if (++index >= 10240) index = 0;
	std::cout << "time = " << sc_time_stamp() << "    value = " << value << std::endl;
	uscita = value;
}


int sc_main (int argc, char *argv[])
{

sc_report_handler::set_actions("/IEEE_Std_1666/deprecated", SC_DO_NOTHING);

sc_set_default_time_unit(1, SC_NS);
sc_clock CLOCK1("clock1",1000,.5,1,true);
sc_clock CLOCK12("clock12",83.33333333,.5,1,true);

sc_core::sc_trace_file *tf = sc_create_vcd_trace_file("simulazione_50_bit");
sc_core::sc_trace_file *f = create_tab_trace_file("simulazione_50_bit", 83.33333333e-9);
sc_core::sc_trace_file *af = create_tab_trace_file("ANALOG",1e-10);

sc_signal<bool> bit_stream_out_SL;
sc_signal<bool> bit_stream_inp;
sc_signal<double> q_out_NCO, q_out_ADC;
sc_signal<double> i_out_NCO, i_out_ADC;
sc_signal<double> out_quad_corr;
sc_signal<double> bit_stream_out;
sc_signal<double> freq, feedback_I, feedback_Q, i_inp_DAC, q_inp_DAC, i_out_DAC, q_out_DAC,i_inp_ADC,q_inp_ADC, I_carry_sync_emulated, Q_carry_sync_emulated, q_out_fir, i_out_fir, i_out_complex_mixer, q_out_complex_mixer, gaussian_out,out_carrier_sinc,bit_stream_out_double, bit_stream_inp_double;
double coeffs[25];
ab_signal <electrical, parallel> transmission(50 ohm,  5e-5, 0), receiving(50 ohm,  1e-7, 0);


bit_generator g1("bit_generator");
	g1.clock2(CLOCK1);
	g1.uscita(bit_stream_inp);

stampxx stampx1("STAMPX1");
	stampx1.inputx(bit_stream_inp);
	stampx1.clk(CLOCK12);
	stampx1.outputx(bit_stream_inp_double);


modulator m1("modulator");
	m1.clk1(CLOCK1);
	m1.clk12(CLOCK12);
	m1.data_in(bit_stream_inp);
	m1.gaussian_out(gaussian_out);
	m1.coos(q_out_NCO);
	m1.siin(i_out_NCO);
DA_conv da("DA_conv");
	da.I_in(i_out_NCO);
	da.Q_in(q_out_NCO);
	da.clock_12MHz(CLOCK12);
	da.I_out(i_out_DAC);
	da.Q_out(q_out_DAC);

TX tx("TX");
tx(transmission,i_out_DAC,q_out_DAC);

// channel with Noise Figure 3dB power KTB con T=290
// DISTANCE, ANTENNA GAIN, NF TX, NF RX, F0 noise, Band noise
radio_channel ch("CH", 5.0, 0.0 dB, 3.0 dB, 3.0 dB, 2.450 GHz, 100 MHz);
ch(transmission,receiving);

RX rx("RX");
rx(receiving,i_inp_ADC,q_inp_ADC);

AD_conv ad("AD_conv");
	ad.I_in(i_inp_ADC);
	ad.Q_in(q_inp_ADC);
	ad.clock_12MHz(CLOCK12);
	ad.I_out(i_out_ADC);
	ad.Q_out(q_out_ADC);



//carry_sync_emulator cse("carry_sync_emulator");
//	cse.clock_12MHz(CLOCK12);
//	cse.siin(I_carry_sync_emulated);
//	cse.coos(Q_carry_sync_emulated);

carrier_sync<double> sync("SYNC", 1e-12, 1e-4, 0.32);
sync(out_quad_corr, bit_stream_out,CLOCK12, out_carrier_sinc);

/*
carrier_sync<double> csy("carrier_sync");
csy.detect_in(out_quad_corr);
csy.detect_out(bit_stream_out);
csy.clock(CLOCK12);
csy.out_freq(out_carrier_sinc);*/




NCO<double> nco_cs("NCO_cs",2e6);
nco_cs.osr_data_stream(out_carrier_sinc);
nco_cs.clock_12MHz(CLOCK12);
nco_cs.siin(I_carry_sync_emulated);
nco_cs.coos(Q_carry_sync_emulated);

	
complex_mixer<double> cm("complex_mixer");
	cm.in_I(i_out_ADC);
	cm.in_Q(q_out_ADC);
	cm.LO_I(I_carry_sync_emulated);
	cm.LO_Q(Q_carry_sync_emulated);
	cm.out_I(i_out_complex_mixer);
	cm.out_Q(q_out_complex_mixer);


  coeffs[0]=0.00870351140888;
  coeffs[1]=0.00753145998572;
  coeffs[2]=0.00388631200119;
  coeffs[3]=-0.00608298148577;
  coeffs[4]=-0.01981903043987;
  coeffs[5]=-0.03060972720885;
  coeffs[6]=-0.02944745974329;
  coeffs[7]=-0.00882684686063;
  coeffs[8]=0.03312594344065;
  coeffs[9]=0.09024676862552;
  coeffs[10]=0.14916885018712;
  coeffs[11]=0.19354089181119;
  coeffs[12]=0.21003730675662;
  coeffs[13]=0.19354089181119;
  coeffs[14]=0.14916885018712;
  coeffs[15]=0.09024676862552;
  coeffs[16]=0.03312594344065;
  coeffs[17]=-0.00882684686063;
  coeffs[18]=-0.02944745974329;
  coeffs[19]=-0.03060972720885;
  coeffs[20]=-0.01981903043987;
  coeffs[21]=-0.00608298148577;
  coeffs[22]=0.00388631200119;
  coeffs[23]=0.00753145998572;
  coeffs[24]=0.00870351140888;


fir<double, 25> fir_I("fir_I",coeffs);
fir_I.clock(CLOCK12);
fir_I.in(i_out_complex_mixer);
fir_I.out(i_out_fir);


fir<double, 25> fir_Q("fir_Q",coeffs);
fir_Q.clock(CLOCK12);
fir_Q.in(q_out_complex_mixer);
fir_Q.out(q_out_fir);


quadricorrelator<double> quad_corr("quadricorrelator");
	quad_corr.in_I(i_out_fir);
	quad_corr.in_Q(q_out_fir);
	quad_corr.clock(CLOCK12);
	quad_corr.out(out_quad_corr);




//demodulator demod("demodulator");
//	demod.in_I(i_dem);
//	demod.in_Q(q_dem);
//	demod.clock(CLOCK1);
//	demod.OSR_clock(CLOCK12);
//	demod.data_stream(bit_str);

decoder<double> dec("decoder");
	dec.in(out_quad_corr);
	dec.clock(CLOCK1);
	dec.OSR_clock(CLOCK12);
	dec.out_data(bit_stream_out);


//carrier_sync<double> sync("SYNC", 1e-6, 1e-6, 0.3);
//sync(out_quad_corr, bit_str,CLOCK12, freq);

// NCO<double> nco("NCO");
// nco(freq, CLOCK12, feedback_I, feedback_Q);


decoder_single_level dsl("DSL");
	dsl.bit_in(bit_stream_out);
	dsl.clock_1MHz(CLOCK1);
	dsl.bit_out(bit_stream_out_SL);

stampxx stampx2("STAMPX2");
	stampx2.inputx(bit_stream_out_SL);
	stampx2.clk(CLOCK12);
	stampx2.outputx(bit_stream_out_double);





sc_trace(tf,bit_stream_inp,"bit_stream_inp");	
sc_trace(tf,bit_stream_out_SL,"bit_stream_out_SL");	
//sc_trace(tf,i_out_NCO,"i_out_NCO");
//sc_trace(tf,q_out_NCO,"q_out_NCO");
//sc_trace(tf,i_out_DAC,"i_out_DAC");
//sc_trace(tf,q_out_DAC,"q_out_DAC");
//sc_trace(tf,i_inp_ADC,"i_inp_ADC");
//sc_trace(tf,q_inp_ADC,"q_inp_ADC");
//sc_trace(tf,i_out_ADC,"i_out_ADC");
//sc_trace(tf,q_out_ADC,"q_out_ADC");
//sc_trace(tf,I_carry_sync_emulated,"I_carry_sync_emulated");
//sc_trace(tf,Q_carry_sync_emulated,"Q_carry_sync_emulated");
//sc_trace(tf,i_out_fir,"i_out_fir");
//sc_trace(tf,q_out_fir,"q_out_fir");
//sc_trace(tf,i_out_complex_mixer,"i_out_complex_mixer");
//sc_trace(tf,q_out_complex_mixer,"q_out_complex_mixer");
//sc_trace(tf,gaussian_out,"gaussian_out");
//sc_trace(tf,out_quad_corr,"out_quad_corr");

f-> trace(bit_stream_inp,"bit_stream_inp");
f-> trace(bit_stream_out,"bit_stream_out");
f-> trace(bit_stream_inp_double,"bit_stream_inp_double");
f-> trace(bit_stream_out_double,"bit_stream_out_double");
f-> trace(I_carry_sync_emulated,"I_carry_sync_emulated");
f-> trace(Q_carry_sync_emulated,"Q_carry_sync_emulated");
f-> trace(out_carrier_sinc,"out_carrier_sinc");
f-> trace(i_out_NCO,"i_out_NCO");
f-> trace(q_out_NCO,"q_out_NCO");
f-> trace(i_out_DAC,"i_out_DAC");
f-> trace(q_out_DAC,"q_out_DAC");
f-> trace(i_inp_ADC,"i_inp_ADC");
f-> trace(q_inp_ADC,"q_inp_ADC");
f-> trace(i_out_ADC,"i_out_ADC");
f-> trace(q_out_ADC,"q_out_ADC");
f-> trace(i_out_fir,"i_out_fir");
f-> trace(q_out_fir,"q_out_fir");
f-> trace(out_quad_corr,"out_quad_corr");
f-> trace(gaussian_out,"gaussian_out");

transmission.trace(af, "CHANNEL_TX");
receiving.trace(af, "CHANNEL_RX");

	sc_core::sc_start(sc_core::sc_time(56001.0,sc_core::SC_NS));

return 0;



}
