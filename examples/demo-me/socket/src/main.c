#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h> // Unbuntu 18 on x86_64 has this at /usr/include/x86_64-linux-gnu/sys/socket.h
#include <signal.h>
#include <pthread.h>
#include <fcntl.h> // for open
#include <unistd.h> // for close
#include <netinet/in.h>
#include <arpa/inet.h> // for inet_addr
#include <sys/time.h>

#include "globals.h"
#include "base_types.h"

//#undef DEBUG_MODE
//#define DEBUG_MODE

//#define DO_INTERACTIVE(x) x
#define DO_INTERACTIVE(x) 

#include "debug.h"
#include "getopt.h"

#include "occgrid.h"    // Occupancy Grid Map Create/Fuse
#include "lz4.h"        // LZ4 Compression/Decompression
#include "xmit_pipe.h"  // IEEE 802.11p WiFi SDR Transmit Pipeline
#include "recv_pipe.h"  // IEEE 802.11p WiFi SDR Receive Pipeline

// The PORTS are defined in the compilation process

char wifi_inet_addr_str[20];

int xmit_sock = 0;
int recv_sock = 0;

char *ack = "OK";

// We will define 2 observations; one "current" and one that is to be constructed to be the new current.
int curr_obs = 1;
int next_obs = 0;
Observation observations[2];

unsigned sock_xmit_count = 0;
unsigned sock_recv_count = 0;

// These variables capture "time" spent in various parts ofthe workload
struct timeval stop_prog, start_prog;

#ifdef INT_TIME
struct timeval stop_pd_lz4_cmp, start_pd_lz4_cmp;
uint64_t pd_lz4_cmp_sec  = 0LL;
uint64_t pd_lz4_cmp_usec = 0LL;

struct timeval stop_pd_wifi_pipe, start_pd_wifi_pipe;
uint64_t pd_wifi_pipe_sec  = 0LL;
uint64_t pd_wifi_pipe_usec = 0LL;

struct timeval stop_pd_wifi_send, start_pd_wifi_send;
uint64_t pd_wifi_send_sec  = 0LL;
uint64_t pd_wifi_send_usec = 0LL;

struct timeval stop_pd_wifi_recv_th, start_pd_wifi_recv_th;
uint64_t pd_wifi_recv_th_sec  = 0LL;
uint64_t pd_wifi_recv_th_usec = 0LL;

struct timeval stop_pd_wifi_lmap_wait, start_pd_wifi_lmap_wait;
uint64_t pd_wifi_lmap_wait_sec  = 0LL;
uint64_t pd_wifi_lmap_wait_usec = 0LL;

struct timeval stop_pd_wifi_recv_wait, start_pd_wifi_recv_wait;
uint64_t pd_wifi_recv_wait_sec  = 0LL;
uint64_t pd_wifi_recv_wait_usec = 0LL;

struct timeval stop_pd_wifi_recv, start_pd_wifi_recv;
uint64_t pd_wifi_recv_sec  = 0LL;
uint64_t pd_wifi_recv_usec = 0LL;

struct timeval stop_pd_recv_pipe, start_pd_recv_pipe;
uint64_t pd_recv_pipe_sec  = 0LL;
uint64_t pd_recv_pipe_usec = 0LL;

struct timeval stop_pd_lz4_uncmp, start_pd_lz4_uncmp;
uint64_t pd_lz4_uncmp_sec  = 0LL;
uint64_t pd_lz4_uncmp_usec = 0LL;

#endif


int counter = 0;
int ascii_counter = 0;

vit_msg_struct_t vit_msg;


// Forward Declarations
void print_usage(char * pname);
void dump_final_run_statistics();
void INThandler(int dummy);
// in globals.h void closeout_and_exit(char* last_msg, int rval);



// Functions, code, etc.
void print_usage(char * pname) {
  printf("Usage: %s <OPTIONS>\n", pname);
  printf(" OPTIONS:\n");
  printf("    -h         : print this helpful usage info\n");
  printf("    -W <str>   : set the internet-address for the WiFi server to <str>\n");
}

void INThandler(int dummy)
{
  printf("In SIGINT INThandler -- Closing the connection and exiting\n");
  closeout_and_exit("Received a SIGINT...", -1);
}


void SIGPIPE_handler(int dummy)
{
  printf("In SIGPIPE_handler -- Closing the connection and exiting\n");
  closeout_and_exit("Received a SIGPIPE...", -1);
}


#ifdef HW_VIT
 extern void init_VIT_HW_ACCEL();
 extern void free_VIT_HW_RESOURCES();
#endif
#ifdef XMIT_HW_FFT
 extern void free_XMIT_FFT_HW_RESOURCES();
#endif
#ifdef RECV_HW_FFT
 extern void free_RECV_FFT_HW_RESOURCES();
#endif

// This cleans up the state before exit
void closeout_and_exit(char* last_msg, int rval)
{
  if (sock_recv_count > 0) {
    dump_final_run_statistics();
  }
  printf("closeout_and_exit -- Closing the connection and exiting %d\n", rval);
  if (xmit_sock != 0) {
    close(xmit_sock);
  }
  if (recv_sock != 0) {
    close(recv_sock);
  }

 #ifdef HW_VIT
  free_VIT_HW_RESOURCES();
 #endif // HW_VIT
 #ifdef XMIT_HW_FFT
  free_XMIT_FFT_HW_RESOURCES();
 #endif
 #ifdef RECV_HW_FFT
  free_RECV_FFT_HW_RESOURCES();
 #endif
  printf("%s\n", last_msg);
  exit(rval);
}


#define MAX_UNCOMPRESSED_DATA_SIZE  sizeof(Costmap2D) // MAX_GRID_SIZE
#define MAX_COMPRESSED_DATA_SIZE    MAX_UNCOMPRESSED_DATA_SIZE //(In case of no compression)?  

#define MAX_XMIT_OUTPUTS  41800  // Really something like 41782 I think

int read_all(int sock, char* buffer, int xfer_in_bytes)
{
  char * ptr;
  int message_size = xfer_in_bytes;
  char* message_ptr = buffer;
  int total_recvd = 0;
  while(total_recvd < message_size) {
    unsigned rem_len = (message_size - total_recvd);
    int valread = read(sock , message_ptr, rem_len);
    message_ptr = message_ptr + valread;
    total_recvd += valread;
    DEBUG2(printf("        read %d bytes for %d total bytes of %d\n", valread, total_recvd, message_size));
    if (valread < 1) {
      DEBUG(printf("  read_all got ZERO bytes -- END of TRANSFER?\n"));
      return total_recvd;
    }
  }
  return total_recvd;
}



int main(int argc, char *argv[])
{
  struct sockaddr_in xmit_servaddr;
  struct sockaddr_in recv_servaddr;
  unsigned char l_buffer[20] = {0};
  //unsigned char buffer[200002] = {0};

  snprintf(wifi_inet_addr_str, 20, "127.0.0.1");

 #ifdef HW_VIT
  DEBUG(printf("Calling init_VIT_HW_ACCEL...\n"));
  init_VIT_HW_ACCEL();
 #endif
  printf("Initializing the OccGrid state...\n");
  init_occgrid_state(); // Initialize the occgrid functions, state, etc.
  printf("Initializing the Transmit pipeline...\n");
  xmit_pipe_init(); // Initialize the IEEE SDR Transmit Pipeline
  printf("Initializing the Receive pipeline...\n");
  recv_pipe_init();

  signal(SIGINT, INThandler);
  signal(SIGPIPE, SIGPIPE_handler);

  // Use getopt to read in run-time options
  // put ':' in the starting of the
  // string so that program can
  // distinguish between '?' and ':'
  int opt;
  while((opt = getopt(argc, argv, ":hB:W:C:s:")) != -1) {
    switch(opt) {
    case 'h':
      print_usage(argv[0]);
      exit(0);
    case 'W':
      snprintf(wifi_inet_addr_str, 20, "%s", optarg);
      break;

    case ':':
      printf("option needs a value\n");
      break;
    case '?':
      printf("unknown option: %c\n", optopt);
      break;
    }
  }

  // Set up the RECV SOCKET SERVER
  printf("Setting up RECV socket server ADR %s and PORT %u\n", wifi_inet_addr_str, RECV_PORT);
  {
    int server_fd, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};
       
    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
      perror("socket failed");
      exit(EXIT_FAILURE);
    }
       
    // Forcefully attaching socket to the port RECV_PORT
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
      perror("setsockopt");
      exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(wifi_inet_addr_str);  /* Set IP address */
    address.sin_port = htons( RECV_PORT );
       
    // Forcefully attaching socket to the port RECV_PORT
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) {
      perror("bind failed");
      exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0) {
      perror("listen");
      exit(EXIT_FAILURE);
    }
    if ((recv_sock = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0)  {
      perror("accept");
      exit(EXIT_FAILURE);
    }
  }
  
  // Set up the XMIT SOCKET SERVER
  printf("Setting up socket server ADDR %s and PORT %u\n", wifi_inet_addr_str, XMIT_PORT);
  {
    int server_fd, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};
       
    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
      perror("socket failed");
      exit(EXIT_FAILURE);
    }
       
    // Forcefully attaching socket to the port XMIT_PORT
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
      perror("setsockopt");
      exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(wifi_inet_addr_str);  /* Set IP address */
    address.sin_port = htons( XMIT_PORT );
       
    // Forcefully attaching socket to the port XMIT_PORT
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) {
      perror("bind failed");
      exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0) {
      perror("listen");
      exit(EXIT_FAILURE);
    }
    if ((xmit_sock = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0)  {
      perror("accept");
      exit(EXIT_FAILURE);
    }
  }

  //#ifdef INT_TIME
  gettimeofday(&start_prog, NULL);
  //#endif

  // This routine loops taking in mesages and generating out Viterbi Decodable message data, until the sockets close/break
  DEBUG(printf("Handling messages...\n"));
  {
    // Now we take in a received transmission with the other AV's map
    // If we receive a transmission, the process to turn it back into the gridMap is:
    //unsigned num_unexpected_messages = 0;
    int      n_recvd_in;
    uint8_t  recvd_in[MAX_XMIT_OUTPUTS];
    while (1) {
     #ifdef INT_TIME
      gettimeofday(&start_pd_wifi_recv_th, NULL);
     #endif
      DEBUG2(printf("\nTrying to Receive data on RECV port %u socket\n", RECV_PORT));
      DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });
     #ifdef INT_TIME
      gettimeofday(&start_pd_wifi_recv_wait, NULL);
     #endif
      char r_buffer[10] = "\0\0\0\0\0\0\0\0\0\0";;
      int valread = read_all(recv_sock, r_buffer, 8);
      DEBUG2(printf("  RECV got %d bytes :'%s'\n", valread, r_buffer));
      if (valread < 1) {
	printf("Closing out the run...\n");
	break;
      } else if (valread == 8) {
	DEBUG2(for (int bi = 0; bi < valread; bi++) {
	    printf("  Byte %4u : 0x%02x  = %c\n", bi, r_buffer[bi], r_buffer[bi]);
	  });
	if(!(r_buffer[0] == 'X' && r_buffer[7] == 'X')) {
	  //printf("ERROR: Unexpected message %u from WiFi...\n", num_unexpected_messages);
	  //num_unexpected_messages++;
	} else {
	  DEBUG(printf("  Received msg %s\n", r_buffer));
	
         #ifdef INT_TIME
	  gettimeofday(&start_pd_wifi_recv, NULL);
         #endif
	  /* if(!(r_buffer[0] == 'X' && r_buffer[7] == 'X')) { */
	  /* 	printf("ERROR: Unexpected message from WiFi...\n"); */
	  /* 	closeout_and_exit("Unexpected WiFi message...", -3); */
	  /* } */

	  char * ptr;
	  unsigned xfer_in_bytes = strtol(r_buffer+1, &ptr, 10);
	  n_recvd_in = xfer_in_bytes / sizeof(uint8_t);
	  DEBUG(printf("     Recv %u UINT8_T values %u bytes from RECV port %u socket\n", n_recvd_in, xfer_in_bytes, RECV_PORT));

	  send(recv_sock, ack, 2, 0);

	  DEBUG(printf("Calling read_all for the %u data-body...\n", sock_recv_count));
	  DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });
	  
         #ifdef INT_TIME
	  gettimeofday(&start_pd_wifi_recv, NULL);
         #endif	
	  valread = read_all(recv_sock, (char*)recvd_in, xfer_in_bytes);
	  if (valread < xfer_in_bytes) {
	    if (valread == 0) {
	      printf("  RECV REAL got ZERO bytes -- END of TRANSFER?\n");
	      closeout_and_exit("RECV REAL got zero bytes..", -1);
	    } else {
	      printf("  RECV REAL got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, xfer_in_bytes);
	      closeout_and_exit("RECV REAL got too few bytes..", -1);
	    }
	  }
	  DEBUG2(printf("XFER %4u : Dumping %u of %u RECV-PIPE bytes\n", sock_recv_count, 32, n_recvd_in);
		for (int i = 0; i < n_recvd_in; i++) {
		  printf("XFER %4u byte %6u : 0x%02x = %c\n", sock_recv_count, i, recvd_in[i], recvd_in[i]);
		}
		printf("\n"));

         #ifdef INT_TIME
	  gettimeofday(&stop_pd_wifi_recv, NULL);
	  pd_wifi_recv_wait_sec  += start_pd_wifi_recv.tv_sec  - start_pd_wifi_recv_wait.tv_sec;
	  pd_wifi_recv_wait_usec += start_pd_wifi_recv.tv_usec - start_pd_wifi_recv_wait.tv_usec;
	  pd_wifi_recv_sec       += stop_pd_wifi_recv.tv_sec  - start_pd_wifi_recv.tv_sec;
	  pd_wifi_recv_usec      += stop_pd_wifi_recv.tv_usec - start_pd_wifi_recv.tv_usec;
         #endif

	  send(recv_sock, ack, 2, 0);

	  DEBUG(printf("RECV %4u : Dumping RECEVIED (map) bytes\n", sock_recv_count);
		for (int i = 0; i < n_recvd_in; i++) {
		  if ((i % 56) == 0) { printf("\n"); }
		  if ((i % 7) == 0) { printf(" "); }
		  printf("%u", recvd_in[i]);
		}
		printf("\n"));
	  
	  DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });
       #if(0)
	  // Now we decompress the grid received via transmission...
	  DEBUG(printf("Calling LZ4_decompress_default...\n"));
	  unsigned char uncmp_data[MAX_UNCOMPRESSED_DATA_SIZE];
         #ifdef INT_TIME
	  gettimeofday(&start_pd_lz4_uncmp, NULL);
         #endif
	  DEBUG(printf("Calling LZ4_decompress_safe with %d input bytes...\n", vit_msg_len));
	  int dec_bytes = LZ4_decompress_safe((char*)recvd_in, (char*)uncmp_data, xfer_in_bytes, MAX_UNCOMPRESSED_DATA_SIZE);
	  if (dec_bytes < 0) {
	    printf("LZ4_decompress_safe ERROR : %d\n", dec_bytes);
	  } DEBUG(else {
	      printf("LZ4_decompress_safe returned %d bytes\n", dec_bytes);
	    });
         #ifdef INT_TIME
	  gettimeofday(&stop_pd_lz4_uncmp, NULL);
	  pd_lz4_uncmp_sec   += stop_pd_lz4_uncmp.tv_sec  - start_pd_lz4_uncmp.tv_sec;
	  pd_lz4_uncmp_usec  += stop_pd_lz4_uncmp.tv_usec - start_pd_lz4_uncmp.tv_usec;
         #endif
       #endif
	  sock_recv_count++;
	  
	  // Now we have the transmission input data to be decoded...
	  //  This is "RAW" data, so we need to set this as a meessage, and call the XMIT and then RECV pipes
	  //  That will generate the Viterbi Decoder input data, which we then send to the other car.

	  if (xfer_in_bytes > 1500) {
	    printf("ERROR: Received too many input bytes -- have to compress the message?\n");
	    closeout_and_exit("RECV got too many input bytes -- add input compression?", -1);
	  }
	  DEBUG(printf("Calling do_xmit_pipeline for %u input grid elements\n", xfer_in_bytes));
	  //DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });
	  int n_xmit_out;
	  float xmit_out_real[MAX_XMIT_OUTPUTS];
	  float xmit_out_imag[MAX_XMIT_OUTPUTS];
         #ifdef INT_TIME
	  gettimeofday(&start_pd_wifi_pipe, NULL);
         #endif	
	  do_xmit_pipeline(xfer_in_bytes, recvd_in, &n_xmit_out, xmit_out_real, xmit_out_imag);
         #ifdef INT_TIME
	  gettimeofday(&stop_pd_wifi_pipe, NULL);
	  pd_wifi_pipe_sec   += stop_pd_wifi_pipe.tv_sec  - start_pd_wifi_pipe.tv_sec;
	  pd_wifi_pipe_usec  += stop_pd_wifi_pipe.tv_usec - start_pd_wifi_pipe.tv_usec;
         #endif
	  DEBUG(printf("  Back from do_xmit_pipeline with %u xmit outputs...\n", n_xmit_out));

	  // Now we send this XMIT_PIPE output through the RECV_PIPE (to get the Viterbi Encoded Data)
	  // FIXME
	  DEBUG(printf("Calling do_recv_pipeline...\n"));
	  //DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });

         #ifdef INT_TIME
	  gettimeofday(&start_pd_recv_pipe, NULL);
         #endif	
	  do_recv_pipeline(n_xmit_out, xmit_out_real, xmit_out_imag, &(vit_msg.len), &(vit_msg.ofdm), &(vit_msg.frame), vit_msg.msg);
         #ifdef INT_TIME
	  gettimeofday(&stop_pd_recv_pipe, NULL);
	  pd_recv_pipe_sec   += stop_pd_recv_pipe.tv_sec  - start_pd_recv_pipe.tv_sec;
	  pd_recv_pipe_usec  += stop_pd_recv_pipe.tv_usec - start_pd_recv_pipe.tv_usec;
         #endif

       #if(0)
	  // And now we can compress to encode for Wifi transmission, etc.
	  unsigned char cmp_data[MAX_COMPRESSED_DATA_SIZE];
         #ifdef INT_TIME
	  gettimeofday(&start_pd_lz4_cmp, NULL);
         #endif	
	  int n_cmp_bytes = LZ4_compress_default((char*)vit_msg, (char*)cmp_data, vit_msg_len, MAX_COMPRESSED_DATA_SIZE);
         #ifdef INT_TIME
	  gettimeofday(&stop_pd_lz4_cmp, NULL);
	  pd_lz4_cmp_sec   += stop_pd_lz4_cmp.tv_sec  - start_pd_lz4_cmp.tv_sec;
	  pd_lz4_cmp_usec  += stop_pd_lz4_cmp.tv_usec - start_pd_lz4_cmp.tv_usec;
         #endif
	  DEBUG(double c_ratio = 100*(1-((double)(n_cmp_bytes)/(double)(MAX_UNCOMPRESSED_DATA_SIZE)));
		printf("  Back from LZ4_compress_default: %lu bytes -> %u bytes for %5.2f%%\n", MAX_UNCOMPRESSED_DATA_SIZE, n_cmp_bytes, c_ratio););
       #endif
    
	  // Now send the resulting Viterbi-Decoder input to the other car...
	  // This is now the content that should be sent out via IEEE 802.11p WiFi
	  //  The n_xmit_out values of xmit_out_real and xmit_out_imag
	  // Connect to the Wifi-Socket and send the n_xmit_out
	  char w_buffer[10];
         #ifdef INT_TIME
	  gettimeofday(&start_pd_wifi_send, NULL);
         #endif	
	  unsigned xfer_bytes = sizeof(struct vit_msg_struct); // vit_msg_len * sizeof(uint8_t);
	  snprintf(w_buffer, 9, "X%-6uX", xfer_bytes);
	  DEBUG(printf("\nXMIT Sending %s on XMIT port %u socket\n", w_buffer, XMIT_PORT));
	  DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });
	  send(xmit_sock, w_buffer, 8, 0);
	  {
	    char isACK[3];
	    int ackread = read_all(xmit_sock, isACK, 2);
	    if ((isACK[0] != ack[0]) || (isACK[1] != ack[1])) {
	      printf("ERROR : Failed to get ACK from receiver after XMIT %u\n", sock_xmit_count);
	      closeout_and_exit("Failed to get an ACK...", -1);
	    } else {
	      DEBUG(printf(" GOT a header %u ACK from the XMIT target\n", sock_xmit_count));
	    }
	  }
	  DEBUG(printf("     Send %u bytes on XMIT port %u socket\n", xfer_bytes, XMIT_PORT));
	  DEBUG2(printf("XFER %4u : Dumping %u of %u XMIT-PIPE raw bytes\n", sock_xmit_count, 32, xfer_bytes);
		 unsigned char* vm_cptr = (char*)(&vit_msg);
		 for (int i = 0; i < 32 /*xfer_bytes*/; i++) {
		   printf("XFER %4u REAL-byte %6u : 0x%02x\n", sock_xmit_count, i, vm_cptr[i]);
		 }
		 printf("\n"));

	  DEBUG(printf("XFER %4u : Sending %u XMIT-PIPE bytes containing\n", sock_xmit_count, xfer_bytes);
		printf("  vit_msg LEN   : %u\n", vit_msg.len);
		printf("  vit_msg OFDM  : \n");
		printf("          ENCODING : %u\n", vit_msg.ofdm.encoding);
		printf("          RATE     : %u\n", vit_msg.ofdm.rate_field);
		printf("          N_BPSC   : %u\n", vit_msg.ofdm.n_bpsc);
		printf("          N_CBPS   : %u\n", vit_msg.ofdm.n_cbps);
		printf("          N_DBPS   : %u\n", vit_msg.ofdm.n_dbps);
		printf("  vit_msg FRAME : \n");
		printf("          PSDU_SIZE   : %u\n", vit_msg.frame.psdu_size);
		printf("          N_SYM       : %u\n", vit_msg.frame.n_sym);
		printf("          N_PAD       : %u\n", vit_msg.frame.n_pad);
		printf("          N_ENC_BITS  : %u\n", vit_msg.frame.n_encoded_bits);
		printf("          N_DATA_BITS : %u\n", vit_msg.frame.n_data_bits);
		printf("  vit_msg MSG-BITS :");
		for (int i = 0; i < 64; i++) {
		  if ((i%8) == 0) {
		    printf("\n          ");
		  }
		  printf("0x%02x ", vit_msg.msg[i]);
		}
		printf("\n"));
	  DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });
	  send(xmit_sock, (char*)(&vit_msg), sizeof(struct vit_msg_struct), 0);
	  DEBUG(printf("     Sent %u bytes on XMIT port %u socket\n", xfer_bytes, XMIT_PORT));
	  {
	    char isACK[3];
	    int ackread = read_all(xmit_sock, isACK, 2);
	    if ((isACK[0] != ack[0]) || (isACK[1] != ack[1])) {
	      printf("ERROR : Failed to get ACK from receiver after XMIT %u\n", sock_xmit_count);
	      closeout_and_exit("Failed to get an ACK...", -1);
	    } else {
	      DEBUG(printf(" GOT a data body %u ACK from the XMIT target\n", sock_xmit_count));
	    }
	  }

         #ifdef INT_TIME
	  gettimeofday(&stop_pd_wifi_send, NULL);
	  pd_wifi_send_sec   += stop_pd_wifi_send.tv_sec  - start_pd_wifi_send.tv_sec;
	  pd_wifi_send_usec  += stop_pd_wifi_send.tv_usec - start_pd_wifi_send.tv_usec;
         #endif
	  DEBUG(printf("Got ACK: Showing first %u bytes of message %u\n", 256, sock_xmit_count);
		for (int i = 0; i < 256; i++) {
		  if ((i % 16) == 0) { printf("\n     "); }
		  printf("%02x ", ((char*)(&vit_msg))[i]);
		});
	  DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });

	  sock_xmit_count++;
	} // if (valread == 8) i.e. we received a raw message to process...
  
       #ifdef INT_TIME
	gettimeofday(&stop_pd_wifi_recv_th, NULL);
	pd_wifi_recv_th_sec  = stop_pd_wifi_recv_th.tv_sec  - start_pd_wifi_recv_th.tv_sec;
	pd_wifi_recv_th_usec = stop_pd_wifi_recv_th.tv_usec - start_pd_wifi_recv_th.tv_usec;
       #endif
      }
    } // while(1);
    printf("Dropped out of while loop -- done!\n");
    // Add one second of wait to see if this helps other cars clear state...
    sleep(1);
  }
  
  dump_final_run_statistics();
  close(xmit_sock);
  close(recv_sock);
}





void dump_final_run_statistics()
{
  printf("\nFinal Run Stats, %u, XMIT, %u, RECV\n", sock_xmit_count, sock_recv_count);
  printf("Occ-Map Dimensions, %u, X, by, %u, Y\n", GRID_MAP_X_DIM, GRID_MAP_Y_DIM);

  printf("Timing (in usec):");
 #ifdef HW_VIT
  printf(" with %u HW_VIT", 1);
 #else 
  printf(" with NO HW_VIT");
 #endif
 #ifdef XMIT_HW_FFT
  printf(" and %u HW_XMIT_FFT", NUM_XMIT_FFT_ACCEL);
 #else 
  printf(" and NO HW_XMIT_FFT");
 #endif
 #ifdef RECV_HW_FFT
  printf(" and %u HW_RECV_FFT", NUM_RECV_FFT_ACCEL);
 #else 
  printf(" and NO HW_RECV_FFT");
 #endif
  printf("\n");

  gettimeofday(&stop_prog, NULL);
  uint64_t total_exec = (uint64_t)(stop_prog.tv_sec - start_prog.tv_sec) * 1000000 + (uint64_t)(stop_prog.tv_usec - start_prog.tv_usec);

 #ifdef INT_TIME
  uint64_t pd_lz4_cmp    = (uint64_t)(pd_lz4_cmp_sec)  * 1000000 + (uint64_t)(pd_lz4_cmp_usec);
  uint64_t pd_wifi_pipe  = (uint64_t)(pd_wifi_pipe_sec)  * 1000000 + (uint64_t)(pd_wifi_pipe_usec);
  uint64_t pd_wifi_send  = (uint64_t)(pd_wifi_send_sec)  * 1000000 + (uint64_t)(pd_wifi_send_usec);
  uint64_t pd_wifi_send_rl = (uint64_t)(pd_wifi_send_sec) * 1000000 + (uint64_t)(pd_wifi_send_usec);
  uint64_t pd_wifi_recv_th   = (uint64_t)(pd_wifi_recv_th_sec)  * 1000000 + (uint64_t)(pd_wifi_recv_th_usec);
  uint64_t pd_wifi_lmap_wait = (uint64_t)(pd_wifi_lmap_wait_sec)  * 1000000 + (uint64_t)(pd_wifi_lmap_wait_usec);
  uint64_t pd_wifi_recv_wait = (uint64_t)(pd_wifi_recv_wait_sec)  * 1000000 + (uint64_t)(pd_wifi_recv_wait_usec);
  uint64_t pd_wifi_recv  = (uint64_t)(pd_wifi_recv_sec) * 1000000 + (uint64_t)(pd_wifi_recv_usec);
  uint64_t pd_recv_pipe  = (uint64_t)(pd_recv_pipe_sec)  * 1000000 + (uint64_t)(pd_recv_pipe_usec);
  uint64_t pd_lz4_uncmp  = (uint64_t)(pd_lz4_uncmp_sec)  * 1000000 + (uint64_t)(pd_lz4_uncmp_usec);

  // This is the xmit_pipe.c breakdown
  uint64_t x_pipe      = (uint64_t)(x_pipe_sec)  * 1000000 + (uint64_t)(x_pipe_usec);
  uint64_t x_genmacfr  = (uint64_t)(x_genmacfr_sec)  * 1000000 + (uint64_t)(x_genmacfr_usec);
  uint64_t x_domapwk   = (uint64_t)(x_domapwk_sec)  * 1000000 + (uint64_t)(x_domapwk_usec);
  uint64_t x_phdrgen   = (uint64_t)(x_phdrgen_sec)  * 1000000 + (uint64_t)(x_phdrgen_usec);
  uint64_t x_ck2sym    = (uint64_t)(x_ck2sym_sec)  * 1000000 + (uint64_t)(x_ck2sym_usec);
  uint64_t x_ocaralloc = (uint64_t)(x_ocaralloc_sec)  * 1000000 + (uint64_t)(x_ocaralloc_usec);
  uint64_t x_fft       = (uint64_t)(x_fft_sec)  * 1000000 + (uint64_t)(x_fft_usec);
  uint64_t x_ocycpref  = (uint64_t)(x_ocycpref_sec)  * 1000000 + (uint64_t)(x_ocycpref_usec);

 #ifdef XMIT_HW_FFT
  uint64_t x_fHtotal   = (uint64_t)(x_fHtotal_sec)  * 1000000 + (uint64_t)(x_fHtotal_usec);
  uint64_t x_fHcvtin   = (uint64_t)(x_fHcvtin_sec)  * 1000000 + (uint64_t)(x_fHcvtin_usec);
  uint64_t x_fHcomp    = (uint64_t)(x_fHcomp_sec)   * 1000000 + (uint64_t)(x_fHcomp_usec);
  uint64_t x_fHcvtout  = (uint64_t)(x_fHcvtout_sec)  * 1000000 + (uint64_t)(x_fHcvtout_usec);
 #endif

  // This is the Xmit doMapWork breakdown
  uint64_t xdmw_total   = (uint64_t)(xdmw_total_sec)   * 1000000 + (uint64_t)(xdmw_total_usec);
  uint64_t xdmw_cnvEnc  = (uint64_t)(xdmw_cnvEnc_sec)  * 1000000 + (uint64_t)(xdmw_cnvEnc_usec);
  uint64_t xdmw_punct   = (uint64_t)(xdmw_punct_sec)   * 1000000 + (uint64_t)(xdmw_punct_usec);
  uint64_t xdmw_intlv   = (uint64_t)(xdmw_intlv_sec)   * 1000000 + (uint64_t)(xdmw_intlv_usec);
  uint64_t xdmw_symbols = (uint64_t)(xdmw_symbls_sec)  * 1000000 + (uint64_t)(xdmw_symbls_usec);
  uint64_t xdmw_mapout  = (uint64_t)(xdmw_mapout_sec)  * 1000000 + (uint64_t)(xdmw_mapout_usec);

  // This is the recv_pipe.c breakdown
  uint64_t r_pipe     = (uint64_t)(r_pipe_sec)  * 1000000 + (uint64_t)(r_pipe_usec);
  uint64_t r_cmpcnj   = (uint64_t)(r_cmpcnj_sec)  * 1000000 + (uint64_t)(r_cmpcnj_usec);
  uint64_t r_cmpmpy   = (uint64_t)(r_cmpmpy_sec)  * 1000000 + (uint64_t)(r_cmpmpy_usec);
  uint64_t r_firc     = (uint64_t)(r_firc_sec)  * 1000000 + (uint64_t)(r_firc_usec);
  uint64_t r_cmpmag   = (uint64_t)(r_cmpmag_sec)  * 1000000 + (uint64_t)(r_cmpmag_usec);
  uint64_t r_cmpmag2  = (uint64_t)(r_cmpmag2_sec)  * 1000000 + (uint64_t)(r_cmpmag2_usec);
  uint64_t r_fir      = (uint64_t)(r_fir_sec)  * 1000000 + (uint64_t)(r_fir_usec);
  uint64_t r_div      = (uint64_t)(r_div_sec)  * 1000000 + (uint64_t)(r_div_usec);
  uint64_t r_sshort   = (uint64_t)(r_sshort_sec)  * 1000000 + (uint64_t)(r_sshort_usec);
  uint64_t r_slong    = (uint64_t)(r_slong_sec)  * 1000000 + (uint64_t)(r_slong_usec);
  uint64_t r_fft      = (uint64_t)(r_fft_sec)  * 1000000 + (uint64_t)(r_fft_usec);
  uint64_t r_eqlz     = (uint64_t)(r_eqlz_sec)  * 1000000 + (uint64_t)(r_eqlz_usec);
  uint64_t r_decsignl = (uint64_t)(r_decsignl_sec)  * 1000000 + (uint64_t)(r_decsignl_usec);
  uint64_t r_descrmbl = (uint64_t)(r_descrmbl_sec)  * 1000000 + (uint64_t)(r_descrmbl_usec);

  // This is the receiver Hardware FFT breakdown
 #ifdef RECV_HW_FFT
  uint64_t r_fHtotal   = (uint64_t)(r_fHtotal_sec)  * 1000000 + (uint64_t)(r_fHtotal_usec);
  uint64_t r_fHcvtin   = (uint64_t)(r_fHcvtin_sec)  * 1000000 + (uint64_t)(r_fHcvtin_usec);
  uint64_t r_fHcomp    = (uint64_t)(r_fHcomp_sec)  * 1000000 + (uint64_t)(r_fHcomp_usec);
  uint64_t r_fHcvtout  = (uint64_t)(r_fHcvtout_sec)  * 1000000 + (uint64_t)(r_fHcvtout_usec);
 #endif

  // This is the sync_short.c "equalize" breakdown
  uint64_t rssh_total    = (uint64_t)(sysh_total_sec)     * 1000000 + (uint64_t)(sysh_total_usec);
  uint64_t rssh_search   = (uint64_t)(sysh_search_sec)    * 1000000 + (uint64_t)(sysh_search_usec);
  uint64_t rssh_frame    = (uint64_t)(sysh_frame_sec)     * 1000000 + (uint64_t)(sysh_frame_usec);
  
  // This is the synch_long.c "equalize" breakdown
  uint64_t rslg_total    = (uint64_t)(sylg_total_sec)    * 1000000 + (uint64_t)(sylg_total_usec);
  uint64_t rslg_firG     = (uint64_t)(sylg_firG_sec)     * 1000000 + (uint64_t)(sylg_firG_usec);
  uint64_t rslg_search   = (uint64_t)(sylg_search_sec)   * 1000000 + (uint64_t)(sylg_search_usec);
  uint64_t rslg_outgen   = (uint64_t)(sylg_outgen_sec)   * 1000000 + (uint64_t)(sylg_outgen_usec);

  // This is the gr_equalizer.c "equalize" breakdown
  uint64_t reql_total    = (uint64_t)(reql_total_sec)     * 1000000 + (uint64_t)(reql_total_usec);
  uint64_t reql_sym_set  = (uint64_t)(reql_symset_sec)    * 1000000 + (uint64_t)(reql_symset_usec);
  uint64_t reql_ls_eql   = (uint64_t)(reql_lseq_call_sec) * 1000000 + (uint64_t)(reql_lseq_call_usec);
  uint64_t reql_out_sym  = (uint64_t)(reql_outsym_sec)    * 1000000 + (uint64_t)(reql_outsym_usec);
  uint64_t reql_ds_fld   = (uint64_t)(reql_decSF_sec)     * 1000000 + (uint64_t)(reql_decSF_usec);
  
  // This is the ofdm.c decode-signal breakdown
  uint64_t rdec_total    = (uint64_t)(rdec_total_sec)  * 1000000 + (uint64_t)(rdec_total_usec);
  uint64_t rdec_map_bitr = (uint64_t)(rdec_map_bitr_sec)  * 1000000 + (uint64_t)(rdec_map_bitr_usec);
  uint64_t rdec_get_bits = (uint64_t)(rdec_get_bits_sec)  * 1000000 + (uint64_t)(rdec_get_bits_usec);
  uint64_t rdec_dec_call = (uint64_t)(rdec_dec_call_sec)  * 1000000 + (uint64_t)(rdec_dec_call_usec);
#endif  
  printf(" Total workload main-loop : %10lu usec\n", total_exec);
#ifdef INT_TIME
  printf("       Total pd lz4_uncmp       : %10lu usec\n", pd_lz4_uncmp);
  printf("       Total pd xmit_pipe       : %10lu usec\n", pd_wifi_pipe);
  printf("         X-Pipe Total Time        : %10lu usec\n", x_pipe);
  printf("         X-Pipe GenMacFr Time     : %10lu usec\n", x_genmacfr);
  printf("         X-Pipe doMapWk Time      : %10lu usec\n", x_domapwk);
  printf("           XdoMW Total Time         : %10lu usec\n", xdmw_total);
  printf("           XdoMW ConvEncode Time    : %10lu usec\n", xdmw_cnvEnc);
  printf("           XdoMW Puncture Time      : %10lu usec\n", xdmw_punct);
  printf("           XdoMW Interleave Time    : %10lu usec\n", xdmw_intlv);
  printf("           XdoMW Gen-Symbols Time   : %10lu usec\n", xdmw_symbols);
  printf("           XdoMW Gen-Map-Out Time   : %10lu usec\n", xdmw_mapout);
  printf("         X-Pipe PckHdrGen Time    : %10lu usec\n", x_phdrgen);
  printf("         X-Pipe Chnk2Sym Time     : %10lu usec\n", x_ck2sym);
  printf("         X-Pipe CarAlloc Time     : %10lu usec\n", x_ocaralloc);
  printf("         X-Pipe Xm-FFT Time       : %10lu usec\n", x_fft);
#ifdef XMIT_HW_FFT
  printf("           X-Pipe xHfft_total Time  : %10lu usec\n", x_fHtotal);
  printf("           X-Pipe xHfft_cvtin Time  : %10lu usec\n", x_fHcvtin);
  printf("           X-Pipe xHfft_comp  Time  : %10lu usec\n", x_fHcomp);
  printf("           X-Pipe xHfft_cvtout Time : %10lu usec\n", x_fHcvtout);
#endif
  printf("         X-Pipe CycPrefix Time    : %10lu usec\n", x_ocycpref);
  printf("       Total pd xmit_send       : %10lu usec\n", pd_wifi_send);
  printf("         Total pd xmit_send_rl    : %10lu usec\n", pd_wifi_send);
  printf("       Total pd xmit_recv_th    : %10lu usec\n", pd_wifi_recv_th);
  printf("         Total pd xmit_lmap_wait  : %10lu usec\n", pd_wifi_lmap_wait);
  printf("         Total pd xmit_recv_wait  : %10lu usec\n", pd_wifi_recv_wait);
  printf("           Total pd xmit_recv       : %10lu usec\n", pd_wifi_recv);
  printf("       Total pd recv_pipe       : %10lu usec\n", pd_recv_pipe);
  printf("         R-Pipe Total Time        : %10lu usec\n", r_pipe);
  printf("         R-Pipe CmplCnjg Time     : %10lu usec\n", r_cmpcnj);
  printf("         R-Pipe CmplMult Time     : %10lu usec\n", r_cmpmpy);
  printf("         R-Pipe FIRC Time         : %10lu usec\n", r_firc);
  printf("         R-Pipe CmplMag Time      : %10lu usec\n", r_cmpmag);
  printf("         R-Pipe CmplMag^2 Time    : %10lu usec\n", r_cmpmag2);
  printf("         R-Pipe FIR Time          : %10lu usec\n", r_fir);
  printf("         R-Pipe DIV Time          : %10lu usec\n", r_div);
  printf("         R-Pipe SyncShort Time    : %10lu usec\n", r_sshort);
  printf("           R-SySht Total Time         : %10lu usec\n", rssh_total);
  printf("           R-SySht Search Time        : %10lu usec\n", rssh_search);
  printf("           R-SySht Frame Time         : %10lu usec\n", rssh_frame);
  printf("         R-Pipe SyncLong Time     : %10lu usec\n", r_slong);
  printf("           R-SyLng Total Time         : %10lu usec\n", rslg_total);
  printf("           R-SyLng FIR-G Time         : %10lu usec\n", rslg_firG);
  printf("           R-SyLng Search Time        : %10lu usec\n", rslg_search);
  printf("           R-SyLng OutGen Time        : %10lu usec\n", rslg_outgen);
  printf("         R-Pipe Rc-FFT Time       : %10lu usec\n", r_fft);
#ifdef RECV_HW_FFT
  printf("           R-Pipe rHfft_total Time  : %10lu usec\n", r_fHtotal);
  printf("           R-Pipe rHfft_cvtin Time  : %10lu usec\n", r_fHcvtin);
  printf("           R-Pipe rHfft_comp  Time  : %10lu usec\n", r_fHcomp);
  printf("           R-Pipe rHfft_cvtout Time : %10lu usec\n", r_fHcvtout);
#endif
  printf("         R-Pipe Equalize Time     :  %10lu usec\n", r_eqlz);
  printf("           R-Eql Total Time         : %10lu usec\n", reql_total);
  printf("           R-Eql Set-Symbol Time    : %10lu usec\n", reql_sym_set);
  printf("           R-Eql LS-EQ Time         : %10lu usec\n", reql_ls_eql);
  printf("           R-Eql Output-Sym Time    : %10lu usec\n", reql_out_sym);
  printf("           R-Eql DecSigFld Time     : %10lu usec\n", reql_ds_fld);
  printf("         R-Pipe DecSignal Time    : %10lu usec\n", r_decsignl);
  printf("           R-Dec Total Time         : %10lu usec\n", rdec_total);
  printf("           R-Dec Map-BitR Time      : %10lu usec\n", rdec_map_bitr);
  printf("           R-Dec Get-Bits Time      : %10lu usec\n", rdec_get_bits);
  printf("           R-Dec Decode Call        : %10lu usec\n", rdec_dec_call);
  printf("         R-Pipe DeScramble Time   : %10lu usec\n", r_descrmbl);
  printf("       Total pd lz4_cmp         : %10lu usec\n", pd_lz4_cmp);
  printf("\n");
 #else
  printf(" NO more detailed timing information on this run...\n");
 #endif
  printf("\nDone with the run...\n");
}
