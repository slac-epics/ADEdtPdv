#include "edtinc.h"
#include "pciload.h"

#define SERBUFSIZE 512

void    print_ascii_string(char *buf);
long    PdvDispatch(int argc, char *argv[], char *edt_devname);
long PdvSerialWriteRead(int argc, char *argv[], int unit, int hexin, int do_wait, int do_printflush, int inter, int timeout, int verbose, int baud, int readonly, int channel);
long    isascii_str(u_char *buf, int n);
long    ack_nak_str(u_char *buf, int n);


int pnum=0;

void printify( u_char *buf, u_char *dest, int n, int show_nonprint )
{
    int     has_lf = FALSE;
    int     i, j;

    for (i = 0; i < n; i++)
    {
        if (buf[i] == 10)
            has_lf = TRUE;
    }

    for (i = 0, j = 0; i < n; i++)
    {

        if (buf[i] == 13)
        {
            if (!has_lf)
            dest[j++] = '\n';
        }
        else if (buf[i] == 10 || (buf[i] >= ' ' && buf[i] < 127))
            dest[j++] = buf[i];
        else if (show_nonprint)
        {
            switch (buf[i])
            {
            case 0x06:
            strcpy((char *) dest + j, "<ACK>");
            break;
            case 0x15:
            strcpy((char *) dest + j, "<NAK>");
            break;
            case 0x02:
            strcpy((char *) dest + j, "<STX>");
            break;
            case 0x03:
            strcpy((char *) dest + j, "<ETX>");
            break;
            default:
            sprintf((char *) dest + j, "<%02x>", buf[i]);
            }

            j = strlen((char *) dest);

        }

        dest[j] = 0;
    }
}


void usage( char * msg )
{
    printf( "%s", msg );
    printf( "Usage: \n" );
    printf(
       "    -?, /?, -h      - Help message\n"
       "    --help          - Help message\n"
       "    -c N            - channel #\n"
       "    --channel N     - channel #\n"
       "    -u N            - Unit number (default 0)\n"
       "    --unit N        - Unit number (default 0)\n"
       "    --readXml       - Read XML GeniCam file and dump to stdout\n"
       "    --reg Addr      - Read Register\n"
       "    -v              - Verbose\n"
    );
}



int main( int argc, char **argv )
{
    long    status;        /* return status from command */
    int     unit = 0;
    int     hexin = 0;
    int     inter = 0;
    int     verbose = 0;
    int     do_wait = 0;
    int     do_printflush = 0;
    int     channel = 0;
    int     readonly = 0;
    int     baud = 0;

    --argc;
    ++argv;
    while ( (argc > 0) && ((argv[0][0] == '-') || (argv[0][0] == '/')) )
    {
        switch (argv[0][1])
		{
		case 'c':
			++argv;
			--argc;
			if (argc < 1)
			{
				usage("Error - channel # expected\n");
				exit(-1);
			}
			channel = atoi(argv[0]);
			break;
		case 'u':        /* device unit number */
			++argv;
			--argc;
			if (argc < 1) 
			{
				usage("Error: option 'u' requires a numeric argument\n");
				exit(1);
			}
			if ((argv[0][0] >= '0') && (argv[0][0] <= '9'))
			{
				unit = atoi(argv[0]);
			}
			else 
			{
				usage("Error: option 'u' requires a numeric argument\n");
				exit(1);
			}
			break;

		case 'v':
			verbose = 1;
			break;

		case '-':
			if ( strcmp( argv[0], "--help" ) == 0 )
			{
				usage( "" );
				exit( 0 );
			}
			else
			{
				fprintf( stderr, "unknown option: %s\n", argv[0] );
				usage( "" );
				exit( 1 );
			}
			break;

		case 'h':
		case '?':
			usage("");
			exit(0);
			break;

		default:
			usage("unknown flag\n");
			exit(1);
			break;
		}
        argc--;
        argv++;
    }

    if ((argc < 1) && !readonly && !do_printflush)
		inter = 1;

    if (readonly && (timeout == 0))
		timeout = 60000;

    status = PdvSerialWriteRead(argc, argv, unit, hexin, do_wait, do_printflush, inter,
            timeout, verbose,  baud, readonly, channel);

    return (0);
}


long
PdvSerialWriteRead(int argc, char *argv[], int unit, int hexin,
          int do_wait, int do_printflush,  int inter, int timeout, int verbose, 
           int baud, int readonly, int channel)
{
    int     i;
    int     ret;
    int     nbytes;
    int     length=0;
    u_char  hbuf[SERBUFSIZE];
    char    tmpbuf[SERBUFSIZE+1];
    char    getbuf[SERBUFSIZE+1];
    char    *ibuf_p;
    u_char  lastbyte, waitc;
    char    buf[SERBUFSIZE+1];
    char    bs[32][3];
    EdtDev *ed;

    /* open a handle to the device     */
    ed = pdv_open_channel(EDT_INTERFACE, unit, channel);
    if (ed == NULL)
    {
        pdv_perror(EDT_INTERFACE);
        return -1;
    }

    pdv_serial_read_enable(ed);

    if (timeout < 1)
    timeout = ed->dd_p->serial_timeout;

    if (verbose)
    printf("serial timeout %d\n", timeout);

    if (inter)
    {
        ibuf_p = getbuf;
        printf("\nEnter command (Ctrl-C to quit)\n\n");
    }
    else
    ibuf_p = argv[0];

    if (baud)
    {
        pdv_set_baud(ed, baud);
    }

    if ((pdv_serial_read(ed, buf, SERBUFSIZE) > 0) && do_printflush)
    {
        printf("%s\n", buf);
        exit (0);
    }

    do
    {
        if (inter)
        {
            printf("> ");
            if (strlen(fgets(ibuf_p,SERBUFSIZE,stdin)) > SERBUFSIZE-1)
            {
                printf("Max command string length (%d) exceeded\n", SERBUFSIZE-1);
                continue;
            }
        }

        if (!readonly)
        {
            if (hexin)
            {
                u_int val;
                i = 0;

                strip_newline(ibuf_p);

                while (*ibuf_p)
                {
                    while ((*ibuf_p == ' ') || (*ibuf_p == '\t'))
                    ++ibuf_p;

                    if (*ibuf_p == '\0')
                    break;

                    if (sscanf(ibuf_p, "%x", &val) != 1)
                    {
                    printf("error reading input byte %d\n",i);
                    i = 0;
                    break;
                    }

                    if (val > 0xff)
                    {
                    printf("Hex string format error -- expect hex bytes separated by spaces, e.g. '00 a0 ff ...' \n");
                    i = 0;
                    break;
                    }
                    else hbuf[i++] = val;

                    while ((*ibuf_p != ' ') && (*ibuf_p != '\t') && (*ibuf_p != '\0'))
                        ++ibuf_p;
                }

                /*
                 * using pdv_serial_binary_command instead of
                 * pdv_serial_write because it prepends a 'c' if FOI
                 */
                if (i)
                    pdv_serial_binary_command(ed, (char *) hbuf, i);
            }
            else if (duncan_framing)
                {
                    nbytes = sscanf(ibuf_p, "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s",
                         bs[0], bs[1], bs[2], bs[3], bs[4], bs[5], bs[6], bs[7],
                            bs[8], bs[9], bs[10], bs[11], bs[12], bs[13], bs[14], bs[15]);

                    for (i = 0; i < nbytes; i++)
                    {
                        if (strlen(bs[i]) > 2)
                        {
                        printf("duncan command format error\n");
                        break;
                        }
                        hbuf[i] = (u_char) (strtoul(bs[i], NULL, 16) & 0xff);
                    }

                    pdv_send_duncan_frame(ed, hbuf, nbytes);
                    /* edt_msleep(10000); */
                }
            else if (basler_framing)
            {
                nbytes = sscanf(ibuf_p, "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s",
                     bs[0], bs[1], bs[2], bs[3], bs[4], bs[5], bs[6], bs[7],
                        bs[8], bs[9], bs[10], bs[11], bs[12], bs[13], bs[14], bs[15]);

                for (i = 0; i < nbytes; i++)
                {
                    if (strlen(bs[i]) > 2)
                    {
                        printf("basler command format error\n");
                        break;
                    }
                    hbuf[i] = (u_char) (strtoul(bs[i], NULL, 16) & 0xff);
                }

                pdv_send_basler_frame(ed, hbuf, nbytes);
                /* edt_msleep(10000); */
            }
            else
            {
                sprintf(tmpbuf, "%s\r", ibuf_p);
                if (verbose)
                    printf("writing <%s>\n", ibuf_p);
                pdv_serial_command(ed, tmpbuf);
            }
        }

        /*
         * serial_timeout comes from the config file (or -t override flag in
         * this app), or if not present defaults to 500 unless readonly
         * defaults to 60000
         */
        if (duncan_framing)
        {
            ret = pdv_read_duncan_frame(ed, (u_char *)buf);
            printf("resp <");
            for (i = 0; i < ret; i++)
                printf("%02x%s", (u_char)buf[i], (i == ret-1)? ">\n" : " ");
        }
        else
        {

            pdv_serial_wait(ed, timeout, 64);

            if (do_wait)
            {
                printf("return to read response: ");
                getchar();
            }

            /*
             * read and print the response. Could be large or small so loop
             * no more characters (OR waitchar seen, if any). Format output
             * for 1) ASCII, 2) HEX, or 3) Pulnix STX/ETX format, as
             * appropriate
             */
            do
            {
                ret = pdv_serial_read(ed, buf, SERBUFSIZE);
                if (verbose)
                    printf("read returned %d\n", ret);

                if (*buf)
                    lastbyte = (u_char)buf[strlen(buf)-1];

                if (ret != 0)
                {
                    buf[ret + 1] = 0;
                    if (hexin || basler_framing || duncan_framing)
                    {
                        int     i;

                        if (ret)
                        {
                            printf("resp <");
                            for (i = 0; i < ret; i++)
                            printf("%s%02x", i ? " " : "", (u_char) buf[i]);
                            printf(">\n");
                        }
                    }
                    else        /* simple ASCII */
                    print_ascii_string(buf);
                    length += ret;
                }

                if (ed->devid == PDVFOI_ID)
                    ret = pdv_serial_wait(ed, 500, 0);
                else if (pdv_get_waitchar(ed, &waitc) && (lastbyte == waitc))
                    ret = 0; /* jump out if waitchar is enabled/received */
                else ret = pdv_serial_wait(ed, 500, 64);
            } while (ret > 0);
        }

        printf("\n");

    } while (inter);

    pdv_close(ed);

    return (1);
}

/*
 * print ascii string. Replace \r or Ctrl-M with \n for consistency and
 * pretty printing
 */
void
print_ascii_string(char *buf)
{
#if 0
    int     i = 0;
    char   *p = buf;
#endif
    char    tmpbuf[SERBUFSIZE];

    printify((u_char *) buf, (u_char *) tmpbuf, strlen(buf), 1);

#if 0
    while (*p)
    {
        if ((*p == '\r' || *p == '\n')
            && (p != buf) && (*(p - 1) != '\n') && (*(p + 1) != '\n'))
            tmpbuf[i++] = '\n';
        else
            tmpbuf[i++] = *p;
        ++p;
    }
    if (tmpbuf[i - 1] != '\n')
        tmpbuf[i++] = '\n';
    tmpbuf[i] = '\0';

#endif
    fputs(tmpbuf, stdout);
    fflush(stdout);
}

long
isascii_str(u_char * buf, int n)
{
    int     i;

    for (i = 0; i < n; i++)
    if ((buf[i] < ' ' || buf[i] > '~')
        && (buf[i] != '\t')
        && (buf[i] != '\n')
        && (buf[i] != '\r'))
        return 0;
    return 1;
}

long
ack_nak_str(u_char *buf, int n)
{
    int     i;

    /* check for all ASCII between the 1st and last chars, or ACK or NAK */
    for (i = 0; i < n; i++)
    if ((buf[i] < ' ' || buf[i] > '~')
        && (buf[i] != 0x15)    /* NAK */
        && (buf[i] != 0x6)    /* ACK */
        && (buf[i] != '\n')    /* LF */
        && (buf[i] != '\r'))/* CR */
        return 0;
    return 1;
}
