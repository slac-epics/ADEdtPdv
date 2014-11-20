
//
// promheader.c -- subroutine and test app to get the EDT device's FPGA PROM header string
//
// TODO: Remove this local copy of an EDT src file for
// future pdv lib versions that provide it

#include "edtinc.h"
#include "pciload.h"

//
// returns the PCI FPGA header string; typically 4 fields (whitespace-separated): Name id date time
// 
char *
get_pci_fpga_header(EdtDev *edt_p, char *name)
{
    u_short stat;
    u_char jumpers, idbits, xidbits;
    int promcode;
    EdtPromData pdata;
    Edt_prominfo *ep;

    promcode = edt_flash_prom_detect(edt_p, &stat);
    ep = edt_get_prominfo(promcode);
    jumpers = stat & 0x3;
    idbits = (stat >>2) & 0x1f;
    xidbits = stat >> 8;

    ep = edt_get_prominfo(promcode);
    edt_read_prom_data(edt_p, promcode, ep->defaultseg, &pdata);

    if (promcode > edt_get_max_promcode())
    {
        printf("  Unknown flash ROM [%02x %02x] - no information available\n", xidbits, idbits);
        return "";
    }

    strcpy(name, pdata.id);
    return name;
}

#define	NO_PROMHEADER_MAIN	1
#ifndef	NO_PROMHEADER_MAIN
// main to test the above subroutine
//
//  specify -u <unit> for other than unit 0
// 
int main(int argc, char **argv)
{
    int     unit = 0, channel = 0;
    char    edt_devname[128];
    char    fpga_name[128];
    char   *progname	= argv[0];
    PdvDev *pdv_p;

    /*
     * process command line arguments
     */
    --argc;
    ++argv;
    while (argc && ((argv[0][0] == '-') || (argv[0][0] == '/')))
    {
        switch (argv[0][1])
        {

            case 'u':		/* device unit number */
                ++argv;
                --argc;
                if (argc < 1) 
                {
                    printf("Error: option 'u' requires an argument\n");
                    exit(1);
                }
                if  ((argv[0][0] >= '0') && (argv[0][0] <= '9'))
                    unit = atoi(argv[0]);
                else strncpy(edt_devname, argv[0], sizeof(edt_devname) - 1);
                break;

            default:
                printf("Usage: %s [-u <unit>]\n (default unit is 0)\n", progname);
                exit(0);
        }
        argc--;
        argv++;
    }

    if ((pdv_p = pdv_open_channel(EDT_INTERFACE, unit, channel)) == NULL)
    {
        char errstr[128];
        sprintf(errstr, "pdv_open_channel(%s%d_%d)", edt_devname, unit, channel);
        pdv_perror(errstr);
        return (1);
    }

    printf("Unit %d boot sector FPGA header: \"%s\"\n", unit, get_pci_fpga_header(pdv_p, fpga_name));
	return 0;
}
#endif	/* NO_PROMHEADER_MAIN */
