
#include <stdlib.h>
#include <errno.h>
#include <curses.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <fcntl.h>	//fcntl

struct hci_request ble_hci_request(uint16_t ocf, int clen, void * status, void * cparam)
{
	struct hci_request rq;
	memset(&rq, 0, sizeof(rq));
	rq.ogf = OGF_LE_CTL;		
	rq.ocf = ocf;
	rq.cparam = cparam;
	rq.clen = clen;
	rq.rparam = status;
	rq.rlen = 1;
	return rq;
}


static int m_nDevice;
static le_set_scan_enable_cp m_scan_cp;
static int m_nBeaconMax;
static char m_BeaconAdr[4][20];

/******************************************************
*	BLE 初期化
*******************************************************/
int Ble_init(int nBeaconMax,char *cpAdr1,char *cpAdr2,char *cpAdr3,char *cpAdr4)
{
	int i,ret, status;

	// Get HCI m_nDevice.

	m_nBeaconMax = nBeaconMax;

	if (nBeaconMax >= 1)	strcpy(&m_BeaconAdr[0][0], cpAdr1);
	if (nBeaconMax >= 2)	strcpy(&m_BeaconAdr[1][0], cpAdr2);
	if (nBeaconMax >= 3)	strcpy(&m_BeaconAdr[2][0], cpAdr3);
	if (nBeaconMax >= 4)	strcpy(&m_BeaconAdr[3][0], cpAdr4);

	m_nDevice = hci_open_dev(hci_get_route(NULL));
	if (m_nDevice < 0) {
		printf("Ble_Init() Failed to open HCI m_nDevice.\n");
		return -1;
	}

	// Set BLE scan parameters.

	le_set_scan_parameters_cp scan_params_cp;
	memset(&scan_params_cp, 0, sizeof(scan_params_cp));
	scan_params_cp.type = 0x00;
	scan_params_cp.interval = htobs(0x0010);
	scan_params_cp.window = htobs(0x0010);
	scan_params_cp.own_bdaddr_type = 0x00; // Public m_nDevice Address (default).
	scan_params_cp.filter = 0x00; // Accept all.

	struct hci_request scan_params_rq = ble_hci_request(OCF_LE_SET_SCAN_PARAMETERS, LE_SET_SCAN_PARAMETERS_CP_SIZE, &status, &scan_params_cp);

	ret = hci_send_req(m_nDevice, &scan_params_rq, 1000);
	if (ret < 0) {
		hci_close_dev(m_nDevice);
		printf("Ble_Init()  Failed to set scan parameters data.\n");
		return -1;
	}

	// Set BLE events report mask.

	le_set_event_mask_cp event_mask_cp;
	memset(&event_mask_cp, 0, sizeof(le_set_event_mask_cp));
	for (i = 0; i < 8; i++) event_mask_cp.mask[i] = 0xFF;

	struct hci_request set_mask_rq = ble_hci_request(OCF_LE_SET_EVENT_MASK, LE_SET_EVENT_MASK_CP_SIZE, &status, &event_mask_cp);
	ret = hci_send_req(m_nDevice, &set_mask_rq, 1000);
	if (ret < 0) {
		hci_close_dev(m_nDevice);
		printf("Ble_Init() Failed to set event mask.\n");
		return -1;
	}

	// Enable scanning.

	memset(&m_scan_cp, 0, sizeof(m_scan_cp));
	m_scan_cp.enable = 0x01;	// Enable flag.
	m_scan_cp.filter_dup = 0x00; // Filtering disabled.

	struct hci_request enable_adv_rq = ble_hci_request(OCF_LE_SET_SCAN_ENABLE, LE_SET_SCAN_ENABLE_CP_SIZE, &status, &m_scan_cp);

	ret = hci_send_req(m_nDevice, &enable_adv_rq, 1000);
	if (ret < 0) {
		hci_close_dev(m_nDevice);
		printf("Ble_Init() Failed to enable scan.\n");
		return 0;
	}

	// Get Results.

	struct hci_filter nf;
	hci_filter_clear(&nf);
	hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
	hci_filter_set_event(EVT_LE_META_EVENT, &nf);
	if (setsockopt(m_nDevice, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0) {
		hci_close_dev(m_nDevice);
		printf("Ble_Init() Could not set socket options\n");
		return 0;
	}


	fcntl(m_nDevice, F_SETFL, O_NONBLOCK);

	printf("Ble_Init() OK\n");
	return 0;
}

/******************************************************
*	BLE RSSI取得
*******************************************************/
int BleRSSI()
{
	int i,len;
	uint8_t buf[HCI_MAX_EVENT_SIZE];
	evt_le_meta_event *meta_event;
	le_advertising_info *info;
	uint8_t reports_count;
	void *offset;
	char addr[18];

	len = read(m_nDevice, buf, sizeof(buf));
	//printf("[%d]",len);
	//fflush(stdout);
	if (len >= HCI_EVENT_HDR_SIZE) {
		meta_event = (evt_le_meta_event*)(buf + HCI_EVENT_HDR_SIZE + 1);
		if (meta_event->subevent == EVT_LE_ADVERTISING_REPORT) {
			reports_count = meta_event->data[0];
			offset = meta_event->data + 1;
			while (reports_count--) {
				info = (le_advertising_info *)offset;
				ba2str(&(info->bdaddr), addr);

				for (i = 0; i < m_nBeaconMax; i++) {
					if (strcmp(addr, &m_BeaconAdr[i][0]) == 0) {
						printf("%s - RSSI %d\n", addr, (int8_t)info->data[info->length]);
					}
				}

				offset = info->data + info->length + 2;
			}
		}
	}

	return 0;
}
/******************************************************
*	BLE 終了処理
*******************************************************/
int Ble_close()
{
	int ret,status;

	memset(&m_scan_cp, 0, sizeof(m_scan_cp));
	m_scan_cp.enable = 0x00;	// Disable flag.

	struct hci_request disable_adv_rq = ble_hci_request(OCF_LE_SET_SCAN_ENABLE, LE_SET_SCAN_ENABLE_CP_SIZE, &status, &m_scan_cp);
	ret = hci_send_req(m_nDevice, &disable_adv_rq, 1000);
	if (ret < 0) {
		hci_close_dev(m_nDevice);
		perror("Failed to disable scan.");
		return 0;
	}

	hci_close_dev(m_nDevice);

	return 0;
}

