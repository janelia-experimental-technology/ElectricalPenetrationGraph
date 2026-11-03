/* Host-side tests for the portable C protocol library.
 *   ./test_proto selftest        internal cobs/crc/i24/frame/parser round-trips
 *   ./test_proto emit  <file>    write known INFO+SAMPLES+EVENT frames (for Python to verify)
 *   ./test_proto parse <file>    parse frames from file, print fields (for Python to check)
 */
#include "../proto/epg_proto.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

/* known fixtures shared with the Python interop test */
static const uint16_t RATES[] = {250, 500, 1000, 2000, 4000};
static const char *SERIAL = "EPG-FW-0001";
static const int32_t SAMP[3][4] = {{100,-200,300,-400},{1,2,3,4},{8388607,-8388608,0,-1}};

static int popcount8(uint8_t x){int c=0;while(x){c+=x&1;x>>=1;}return c;}

/* ---- selftest ---- */
static int fails = 0;
#define CHECK(c, msg) do{ if(!(c)){ printf("  FAIL %s\n", msg); fails++; } else printf("  PASS %s\n", msg);}while(0)

static void test_cobs(void){
    const uint8_t cases[][8] = {{0},{0,0},{1,2,3},{0,1,0,2}};
    const size_t lens[] = {1,2,3,4};
    int allok = 1;
    for (int i=0;i<4;i++){
        uint8_t enc[64], dec[64]; int ok;
        size_t e = epg_cobs_encode(cases[i], lens[i], enc);
        for (size_t k=0;k<e;k++) if (enc[k]==0) allok=0; /* no zeros in output */
        size_t d = epg_cobs_decode(enc, e, dec, &ok);
        if (!ok || d!=lens[i] || memcmp(dec,cases[i],d)) allok=0;
    }
    CHECK(allok, "cobs roundtrip + no zero bytes");
}
static void test_crc(void){
    uint16_t c = epg_crc16((const uint8_t*)"123456789", 9);
    CHECK(c == epg_crc16((const uint8_t*)"123456789",9), "crc deterministic");
    CHECK(c != epg_crc16((const uint8_t*)"123456780",9), "crc content-sensitive");
}
static void test_i24(void){
    int32_t vs[] = {0,1,-1,8388607,-8388608,12345,-999999};
    int ok=1; uint8_t b[3];
    for (int i=0;i<7;i++){ epg_pack_i24(vs[i], b); if (epg_unpack_i24(b)!=vs[i]) ok=0; }
    CHECK(ok, "i24 roundtrip");
}
static int g_seen; static uint8_t g_type;
static void cb(uint8_t t, uint8_t f, const uint8_t*p, size_t n, void*u){
    (void)f;(void)p;(void)u; g_seen++; g_type=t; (void)n;
}
static void test_frame_parser(void){
    uint8_t payload[] = {0x10,0x00,0x20,0xFF}, frame[64];
    size_t fl = epg_encode_frame(T_EVENT, 3, payload, sizeof payload, frame);
    CHECK(frame[fl-1]==0x00, "frame ends with delimiter");
    epg_parser_t pp; epg_parser_init(&pp, cb, NULL); g_seen=0;
    epg_parser_feed(&pp, frame, fl);
    CHECK(g_seen==1 && g_type==T_EVENT, "parser delivers one EVENT frame");
    /* corrupt a byte -> CRC error, parser stays alive for next clean frame */
    uint8_t bad[64]; memcpy(bad,frame,fl); bad[2]^=0xFF;
    uint8_t frame2[64]; size_t fl2 = epg_encode_frame(T_PING,0,NULL,0,frame2);
    epg_parser_init(&pp, cb, NULL); g_seen=0;
    epg_parser_feed(&pp, bad, fl); epg_parser_feed(&pp, frame2, fl2);
    CHECK(pp.crc_errors>=1 && g_seen>=1, "parser flags CRC error and resyncs");
}
/* A realistic 50 ms @ 4 kHz x 8 ch sample block (4.8 KB) must encode and decode intact. */
static void test_large_frame(void){
    enum { ROWS = 200, NCH = 8 };
    static uint8_t pay[17 + ROWS*NCH*3], frame[EPG_FRAME_MAX(sizeof pay)], dec[sizeof pay + 8];
    size_t pl = epg_build_sampleblock_header(1,0,4,0xFF,ROWS,0,pay);
    for (int i=0;i<ROWS*NCH;i++){ epg_pack_i24((i*7919 % 16000000) - 8000000, pay+pl); pl+=3; }
    size_t fl = epg_encode_frame(T_SAMPLES,0,pay,pl,frame);
    CHECK(fl <= EPG_FRAME_MAX(pl) && frame[fl-1]==0, "large frame fits EPG_FRAME_MAX and is delimited");
    int zeros=0; for (size_t i=0;i<fl-1;i++) if(frame[i]==0) zeros++;
    CHECK(zeros==0, "no 0x00 inside a large COBS frame");
    int ok=0; size_t bl = epg_cobs_decode(frame, fl-1, dec, &ok);
    CHECK(ok && bl==pl+4, "large frame COBS decodes to type+flags+payload+crc");
    uint16_t got = dec[bl-2] | (dec[bl-1]<<8);
    CHECK(got==epg_crc16(dec,bl-2) && dec[0]==T_SAMPLES && memcmp(dec+2,pay,pl)==0,
          "large frame CRC and payload intact");
}
static int selftest(void){
    printf("C protocol selftest:\n");
    test_cobs(); test_crc(); test_i24(); test_frame_parser(); test_large_frame();
    if (fails){ printf("FAILED (%d)\n", fails); return 1; }
    printf("All C protocol tests passed.\n"); return 0;
}

/* ---- emit known frames ---- */
static int emit(const char *path){
    FILE *f = fopen(path,"wb"); if(!f){perror("open");return 1;}
    uint8_t pay[512]; uint8_t frame[EPG_FRAME_MAX(512)]; size_t pl, fl;

    epg_info_t info = { EPG_PROTO_VERSION, 0,1,0, 8, 1200,800,1000,16,24, RATES,5, SERIAL };
    pl = epg_build_info(&info, pay);
    fl = epg_encode_frame(T_INFO,0,pay,pl,frame); fwrite(frame,1,fl,f);

    pl = epg_build_sampleblock_header(7,1000,2,0x0F,3,0,pay);
    for (int r=0;r<3;r++) for (int c=0;c<4;c++){ epg_pack_i24(SAMP[r][c], pay+pl); pl+=3; }
    fl = epg_encode_frame(T_SAMPLES,0,pay,pl,frame); fwrite(frame,1,fl,f);

    pl = epg_build_event(EV_VS_CHANGE,2,54321,32768,40000,"track",5,pay);
    fl = epg_encode_frame(T_EVENT,0,pay,pl,frame); fwrite(frame,1,fl,f);
    /* large block: 200 rows x 8 ch (what the firmware sends at 4 kHz / 50 ms) */
    {
        static uint8_t bp[17 + 200*8*3], bf[EPG_FRAME_MAX(sizeof bp)];
        size_t bl = epg_build_sampleblock_header(8,5000,4,0xFF,200,0,bp);
        for (int i=0;i<200*8;i++){ epg_pack_i24(((i*7919) % 16000000) - 8000000, bp+bl); bl+=3; }
        size_t bfl = epg_encode_frame(T_SAMPLES,0,bp,bl,bf); fwrite(bf,1,bfl,f);
    }
    fclose(f); return 0;
}

/* ---- parse frames and print fields ---- */
static void pcb(uint8_t t, uint8_t flags, const uint8_t*p, size_t n, void*u){
    (void)flags;(void)u;
    if (t==T_INFO){
        uint8_t nch=p[4], nrates=p[5];
        uint16_t vref=p[6]|(p[7]<<8), gain=p[8]|(p[9]<<8);
        size_t off = 15 + 2*nrates; uint8_t slen=p[off];
        char s[40]; memcpy(s,p+off+1,slen); s[slen]=0;
        printf("INFO proto=%u nch=%u nrates=%u vref=%u gain=%u serial=%s\n",
               p[0],nch,nrates,vref,gain,s);
    } else if (t==T_SAMPLES){
        uint32_t seq = p[0]|(p[1]<<8)|(p[2]<<16)|((uint32_t)p[3]<<24);
        uint64_t first=0; for(int i=0;i<8;i++) first|=((uint64_t)p[4+i])<<(8*i);
        uint8_t mask=p[13]; uint16_t ns=p[14]|(p[15]<<8);
        int nchp = popcount8(mask);
        const uint8_t *d = p+17;
        int32_t s0c0 = epg_unpack_i24(d);
        int32_t last = epg_unpack_i24(d + 3*(nchp*ns - 1));
        printf("SAMPLES seq=%u first=%llu mask=0x%02x n=%u s0c0=%d last=%d\n",
               seq,(unsigned long long)first,mask,ns,s0c0,last);
    } else if (t==T_EVENT){
        uint64_t idx=0; for(int i=0;i<8;i++) idx|=((uint64_t)p[2+i])<<(8*i);
        int32_t a=(int32_t)(p[10]|(p[11]<<8)|(p[12]<<16)|((uint32_t)p[13]<<24));
        int32_t b=(int32_t)(p[14]|(p[15]<<8)|(p[16]<<16)|((uint32_t)p[17]<<24));
        uint8_t tl=p[18]; char s[64]; memcpy(s,p+19,tl); s[tl]=0;
        printf("EVENT type=%u ch=%u idx=%llu a=%d b=%d text=%s\n",
               p[0],p[1],(unsigned long long)idx,a,b,s);
    }
    (void)n;
}
static int parse_file(const char *path){
    FILE *f=fopen(path,"rb"); if(!f){perror("open");return 1;}
    uint8_t buf[4096]; size_t n=fread(buf,1,sizeof buf,f); fclose(f);
    epg_parser_t pp; epg_parser_init(&pp,pcb,NULL);
    epg_parser_feed(&pp,buf,n);
    return 0;
}

int main(int argc, char**argv){
    if (argc>=2 && !strcmp(argv[1],"selftest")) return selftest();
    if (argc>=3 && !strcmp(argv[1],"emit"))  return emit(argv[2]);
    if (argc>=3 && !strcmp(argv[1],"parse")) return parse_file(argv[2]);
    fprintf(stderr,"usage: %s selftest|emit <f>|parse <f>\n", argv[0]); return 2;
}
