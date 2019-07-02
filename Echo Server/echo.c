// Modified LWIP Echo server for sending multiple image frame data to an FPGA for processing
// Developing authors: Atiyeh Panahi, Keaten Stokke

#include <stdio.h>
#include <string.h>
#include "xbram_hw.h"
#include "xparameters.h"
#include <stdlib.h>
#include "lwip/err.h"
#include "lwip/tcp.h"
#if defined (__arm__) || defined (__aarch64__)
#include "xil_printf.h"
#include "xtime_l.h"
#endif

int transfer_data()
{
	return 0;
}

void print_app_header()
{
#if (LWIP_IPV6==0)
	xil_printf("\n\r\n\r-----lwIP TCP echo server ------\n\r");
#else
	xil_printf("\n\r\n\r-----lwIPv6 TCP echo server ------\n\r");
#endif
	xil_printf("TCP packets sent to port 7 will be echoed back\n\r");
}

// Converts a string to an integer
char* my_itoa(int i, char b[])
{
    char const digit[] = "0123456789";
    char* p = b;
	if(i<0)
	{
        *p++ = '-';
        i *= -1;
    }

    int shifter = i;

    do
    { //Move to where representation ends
        ++p;
        shifter = shifter/10;
    }
    while(shifter);

    *p = '\0';

    do
    { //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }
    while(i);

    return b;
}

err_t recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
	int Data_out_Echo_1, Data_out_Echo_2, Data_out_Echo_3;
	int Data_in, Data_in_param, Data_out_param, Data_out_subset, Data_out_gamma;
	static int frame_counter = 0;
	static int packet_counter = 0;
	static int pixel_counter = 0;
	static int param_counter = 0;
	static int results_cnt = 1;
	static int num_subsets = 0;
	static int print_counter_1 = 0;
	char* pixel = "";
	char* param = "";
	char* string_payload;
	char* string_Data_out_Echo_1;
	char* string_Data_out_Echo_2;
	char* string_Data_out_Echo_3;
	char* concatted_out = "";
	int i = 0;
	int j = 0;
	int i_param = 0;
	int j_param = 0;
	// Pixel count based on frame size (232x448 = 103,936)
	int frame_size = 103936;

	// Memory-mapped address space
	static long long int address = 0x00B0000000; 							// Alternating address (Initialized to BRAM_ctrl_0 Base Address)
	static long long int bram_address_0 = 0x00B0000000; 					// BRAM_ctrl_0 Base Address
	static long long int bram_address_1 = 0x00B0080000; 					// BRAM_ctrl_1 Base Address
	static long long int results_address = 0x00B0100000; 					// BRAM_ctrl_2 Base Address
	static long long int address_param = 0x00B0101000; 		    			// BRAM_ctrl_3 Base Address
	static long long int address_subset_param = 0x00B0102000; 				// BRAM_ctrl_4 Base Address
	static long long int address_gamma_param = 0x00B0103000; 				// BRAM_ctrl_5 Base Address
	long long int BaseAddress_interface = 0x00A0006000;						// Interface IP Base Address
	long long int BaseAddress_Grad = 0x00A0005000;							// Gradients IP Base Address
	long long int BaseAddress_Param = 0x00A0003000;							// Parameters IP Base Address
	long long int BaseAddress_Gamma = 0x00A0001000;							// Gamma IP Base Address
	long long int BaseAddress_Results = 0x00A0004000;						// Results IP Base Address
	long long int RegOffset_interface_reg0 = 0x0000000000;					// AXI slv_reg0 Base Address
	long long int RegOffset_interface_reg1 = 0x0000000004;					// AXI slv_reg1 Base Address

	char buffer_1 [32];
	char buffer_2 [32];
	char buffer_3 [32];

	// Timing
	XTime start_print, end_print;

	//xil_printf("in recv_callback End = %d \n\r", XBram_ReadReg(BaseAddress_Results, RegOffset_interface_reg0));
	//xil_printf("p->len = %d \n\r", p->len);

	/* do not read the packet if we are not in ESTABLISHED state */
	if(!p)
	{
		tcp_close(tpcb);
		tcp_recv(tpcb, NULL);
		return ERR_OK;
	}

	/* indicate that the packet has been received */
	tcp_recved(tpcb, p->len);

	/* echo back the payload */
	/* in this case, we assume that the payload is < TCP_SND_BUF */
	if(tcp_sndbuf(tpcb) > p->len)
	{
		string_payload = p->payload;

		// If the payload received is of length 7 then the Python script sent "Results" and is ready to receive them
		if(p->len == 7)
		{
			print_counter_1++;
			xil_printf("frame_counter = %d \n\r", frame_counter);

			print_counter_1++;
			xil_printf("num_subsets = %d \n\r", num_subsets);
			if(results_cnt <= ((frame_counter - 1) * num_subsets))
			{
				// Reads all of the results from the corresponding BRAM to send to the PC
				//print_counter_1++;
				//xil_printf("Results \n\r");
				results_address = results_address + 4;
				Data_out_Echo_1 = XBram_In32(results_address);
				results_address = results_address + 4;
				Data_out_Echo_2 = XBram_In32(results_address);
				results_address = results_address + 4;
				Data_out_Echo_3 = XBram_In32(results_address);

				// Calls the format function so each result is of length 10 so the Python script can break them apart
				string_Data_out_Echo_1 = my_itoa(Data_out_Echo_1, buffer_1);
				string_Data_out_Echo_2 = my_itoa(Data_out_Echo_2, buffer_2);
				string_Data_out_Echo_3 = my_itoa(Data_out_Echo_3, buffer_3);

				// Resets the string we send back to the PC
				concatted_out = "";

				// Concat's all of the results together to send as 1 packet (per frame/subset) to the PC
				if(results_cnt == ((frame_counter - 1) * num_subsets))
				{
					strcpy(concatted_out, string_Data_out_Echo_1);
					strcat(concatted_out, ",");
					strcat(concatted_out, string_Data_out_Echo_2);
					strcat(concatted_out, ",");
					strcat(concatted_out, string_Data_out_Echo_3);
				}
				else
				{
					strcpy(concatted_out, string_Data_out_Echo_1);
					strcat(concatted_out, ",");
					strcat(concatted_out, string_Data_out_Echo_2);
					strcat(concatted_out, ",");
					strcat(concatted_out, string_Data_out_Echo_3);
					strcat(concatted_out, ",");
				}

				results_cnt = results_cnt + 1;
				err = tcp_write(tpcb, concatted_out, strlen(concatted_out), 1);
			}
			else
			{
				// Tells the Python script that the echo server is finished sending data
				print_counter_1++;
				XTime_StartTimer();
				XTime_GetTime(&start_print);
				xil_printf("Finished \n\r");
				XTime_GetTime(&end_print);
				printf("printf took %lu clock cycles.\n", 2*(end_print - start_print));

				print_counter_1++;
				xil_printf("Total Prints = %d \n\r", print_counter_1);
				err = tcp_write(tpcb, "Finished", 8, 1);
			}

		}
		// Python script informs the echo server that it is finished sending data
		else if(p->len == 5)
		{
			if(XBram_ReadReg(BaseAddress_Results, RegOffset_interface_reg0) == 0)
			{
				// Setting new_frame to 2
				XBram_WriteReg(BaseAddress_Gamma, RegOffset_interface_reg0, 2);
				print_counter_1++;
				xil_printf("Done \n\r");
				err = tcp_write(tpcb, "Done!", 5, 1);
			}
			else
			{
				print_counter_1++;
				xil_printf("Okay \n\r");
				err = tcp_write(tpcb, "Okay!", 5, 1);
			}
		}
		// Python script is waiting to send a new frame, so it sends a "wait" until Gamma is done processing
		else if(p->len == 4)
		{
			// If Gamma is not finished, keep waiting
			if(XBram_ReadReg(BaseAddress_Gamma, RegOffset_interface_reg1) == 0)
			{
				print_counter_1++;
				xil_printf("Wait \n\r");
				err = tcp_write(tpcb, "Wait", 4, 1);
			}
			// If Gamma is finished, send a "send" to request a new frame from the Python script
			else if(XBram_ReadReg(BaseAddress_Gamma, RegOffset_interface_reg1) == 1)
			{
				print_counter_1++;
				xil_printf("Send \n\r");

				// Activates Gamma
				XBram_WriteReg(BaseAddress_Gamma, RegOffset_interface_reg0, 1);

				// Deactivates  Gamma
				XBram_WriteReg(BaseAddress_Gamma, RegOffset_interface_reg0, 0);
				err = tcp_write(tpcb, "Send", 4, 1);
			}
		}
		else
		{
			// Echos back the image data to the Python script
			err = tcp_write(tpcb, p->payload, p->len, 1);
		}
		// Writing the received payload (packet of frame data) to the corresponding FPGA memory space
		while(i < p->len)
		{
			// Saving the parameters
			if(packet_counter == 0)
			{
				// Get the packet, split the data on every 10th position, write the to an address and increment the address
				while(i_param < p->len)
				{
					param[j_param] = string_payload[i_param];
					j_param++;
					i_param++;
					if(i_param % 10 == 0)
					{
						Data_in_param = atoi(param);
						XBram_Out32(address_param, Data_in_param);
						XBram_Out32(address_subset_param, Data_in_param);
						XBram_Out32(address_gamma_param, Data_in_param);

						address_param = address_param + 4;
						address_subset_param = address_subset_param + 4;
						address_gamma_param = address_gamma_param + 4;

						// Sets the number of subsets to the local variable
						if(param_counter == 4)
						{
							num_subsets = Data_in_param;
						}

						j_param = 0;
						param_counter++;
					}
				}
				XBram_WriteReg(BaseAddress_Param, RegOffset_interface_reg0, 1);
				packet_counter = 1;
			}
			else
			{
				pixel[j] = string_payload[i];
				j++;
				i++;
				if(i % 10 == 0)
				{
					Data_in = atoi(pixel);
					XBram_Out32(address, Data_in);
					address = address + 4;
					j = 0;
					pixel_counter++;

					// Count the number frames received and notify the FPGA that a new frame has been saved
					if(pixel_counter % frame_size == 0)
					{
						frame_counter++;
						if(frame_counter == 1)
						{
							XBram_WriteReg(BaseAddress_Grad, RegOffset_interface_reg1, frame_counter);
						}
						else if(frame_counter == 2)
						{
							// Updates/flips the MUX
							XBram_WriteReg(BaseAddress_Grad, RegOffset_interface_reg1, frame_counter);

							// Activates the interface IP for gradients and gamma
							XBram_WriteReg(BaseAddress_interface, RegOffset_interface_reg0, 1);

							// Setting new_frame = 1 (turning gradients on then off so it starts but doesn't iterate again)
							XBram_WriteReg(BaseAddress_Grad, RegOffset_interface_reg0, 1);
							XBram_WriteReg(BaseAddress_Grad, RegOffset_interface_reg0, 0);
						}
						else if(frame_counter > 2)
						{
							// Updates/flips the MUX
							XBram_WriteReg(BaseAddress_Grad, RegOffset_interface_reg1, frame_counter);

							// Setting new_frame = 1 (turning gradients on then off so it starts but doesn't iterate again)
							XBram_WriteReg(BaseAddress_Grad, RegOffset_interface_reg0, 1);
							XBram_WriteReg(BaseAddress_Grad, RegOffset_interface_reg0, 0);
						}
					}
				}
			}
		}
		// If we reached the last packet of the frame, switch which BRAM we write the frame to
		if(p->len == 1120) //for 232*448, with bsp = 1440
		{
			if(frame_counter % 2 == 1)
			{
				address = bram_address_1; 		// BRAM_ctrl_1 Address; next frame goes to bram_ctrl_0
			}
			else if(frame_counter % 2 == 0)
			{
				address = bram_address_0; 		// BRAM_ctrl_0 Address; next frame goes to bram_ctrl_1
			}
		}
	}
	else
	{
		xil_printf("no space in tcp_sndbuf\n\r");
	}

	/* free the received pbuf */
	pbuf_free(p);

	return ERR_OK;
}

err_t accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
	static int connection = 1;

	/* set the receive callback for this connection */
	tcp_recv(newpcb, recv_callback);

	/* just use an integer number indicating the connection id as the
	   callback argument */
	tcp_arg(newpcb, (void*)(UINTPTR)connection);

	/* increment for subsequent accepted connections */
	connection++;

	return ERR_OK;
}


int start_application()
{
	struct tcp_pcb *pcb;
	err_t err;
	unsigned port = 7;

	/* create new TCP PCB structure */
	pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
	if (!pcb)
	{
		xil_printf("Error creating PCB. Out of Memory\n\r");
		return -1;
	}

	/* bind to specified @port */
	err = tcp_bind(pcb, IP_ANY_TYPE, port);
	if (err != ERR_OK)
	{
		xil_printf("Unable to bind to port %d: err = %d\n\r", port, err);
		return -2;
	}

	/* we do not need any arguments to callback functions */
	tcp_arg(pcb, NULL);

	/* listen for connections */
	pcb = tcp_listen(pcb);
	if (!pcb)
	{
		xil_printf("Out of memory while tcp_listen\n\r");
		return -3;
	}

	/* specify callback to use for incoming connections */
	tcp_accept(pcb, accept_callback);

	xil_printf("TCP echo server started @ port %d\n\r", port);

	return 0;
}
