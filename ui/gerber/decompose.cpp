#include <stdint.h>

#include <cstdio>
#include <string>
#include <vector>

class Vec2 {
    public:
        double x, y;

};

class RVec2I {
    public:
        int r, theta;
};

class Box2I {
    public:
        Box2I()
        {
            x=y=w=h=0;
        }

        void Show()
        {
            printf("%d %d - (%d %d)\n", x, y, w, h );
        }

        int x, y, w, h;

};

class MonoBitmap
{
    public:
        MonoBitmap(int w, int h, int stride = 0)
        {
            m_w = w;
            m_h = h;

	    if(stride==0)
		m_stride = (w+7)/8;
	    else
		m_stride = stride;

            m_data = new uint8_t[m_stride * h];

        }

        ~MonoBitmap()
        {
            delete m_data;
        }

        static MonoBitmap *LoadFromFile ( const std::string& filename );

        int GetPixel ( int x, int y ) const
        {
            if(x<0 || y<0 || x>=m_w || y >=m_h)
                return 0;

            return m_data[m_stride * y + (x>>3)] & (1<<(x&0x7)) ? 1 : 0;
        }

        void SetPixel ( int x, int y, int v )
        {
            if(x<0 || y<0 || x>=m_w || y >=m_h)
                return;

            if(v)
                m_data[m_w/8 * y + (x>>3)] |= (1<<(x&0x7));
            else
                m_data[m_w/8 * y + (x>>3)] &= ~(1<<(x&0x7));

        }

        void SetResolution( double res )
        {
            m_resolution = res;
        }

        double GetResolution() const
        {
            return m_resolution;
        }

    	int Width() { return m_w; }
        int Height() { return m_h; }

        Box2I NonzeroArea ();

//  private:
        int m_w, m_h, m_stride;
        uint8_t *m_data;
        double m_resolution;
};

class EtchSpot {


private:


};

class EtchEnvironment
{
    public:
        void SetOriginOffset( const Vec2& offset );
        void SetDeadzone ( double deadZone );
        void SetSpotSize ( double spotSize );
        void SetThetaScale ( double oneDegree );
        void SetRUnit ( double rUnitInMM );


        float SpotCoverage ( const EtchSpot& spot );

        void Process();


    private:



};

static uint8_t revbits( uint8_t x )
{
    static const unsigned char BitReverseTable256[] =
{
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

    return BitReverseTable256[x];
}

MonoBitmap *MonoBitmap::LoadFromFile ( const std::string& filename )
{
    FILE *f = fopen(filename.c_str(), "rb");

    struct PcxHeader
    {
        uint8_t Identifier;        // PCX Id Number (Always 0x0A) // 10
        uint8_t Version;           // Version Number    // 5
        uint8_t Encoding;          // Encoding Format    // 1
        uint8_t BitsPerPixel;      // Bits per Pixel    // 8
        uint16_t XStart;            // Left of image     // 0
        uint16_t YStart;            // Top of Image     // 0
        uint16_t XEnd;              // Right of Image    // 319
        uint16_t YEnd;              // Bottom of image    // 199
        uint16_t HorzRes;           // Horizontal Resolution   // 320
        uint16_t VertRes;           // Vertical Resolution   // 200
        uint8_t Palette[48];       // 16-Color EGA Palette
        uint8_t Reserved1;         // Reserved (Always 0)
        uint8_t NumBitPlanes;      // Number of Bit Planes   // 1
        uint16_t bytesPerLine;      // uint8_ts per Scan-line   // 320
        uint16_t PaletteType;       // Palette Type     // 0
        uint16_t HorzScreenSize;    // Horizontal Screen Size   // 0
        uint16_t VertScreenSize;    // Vertical Screen Size   // 0
        uint8_t Reserved2[54];     // Reserved (Always 0)
    } __attribute__((packed)) ;

    PcxHeader hdr;
    fread(&hdr, sizeof(PcxHeader), 1, f);
    printf("Resolution [pix per inch] %d x %d\n", hdr.HorzRes, hdr.VertRes);
    printf("Size: %d x %d bpl %d \n", hdr.XEnd + 1, hdr.YEnd + 1, hdr.bytesPerLine);



    int w = hdr.XEnd + 1;
    int h = hdr.YEnd + 1;

    int pixelCount = 8 * hdr.bytesPerLine * h;

    MonoBitmap *bmp = new MonoBitmap ( w, h, hdr.bytesPerLine );
    int idx = 0;

    bmp->SetResolution( (double)hdr.HorzRes );

    while (pixelCount > 0)
    {
        uint8_t b;

        if(feof(f))
            break;
        fread(&b, 1, 1, f);


        if ((b & 0xc0) == 0xc0)
        {
            int count = b & 0x3F;
            fread ( &b, 1, 1, f);


            //printf("cnt %d col %x\n", count, b);
            for ( int i=0;i<count;i++)
            {
                if(pixelCount == 0)
                    break;

                pixelCount -= 8;
                bmp->m_data[idx++] = revbits(b);

            }


        } else {
            //printf("nonrle %x\n", b);
            pixelCount -= 8;
            bmp->m_data[idx++] = revbits(b & 0x3f);
        }
    }

    return bmp;
}

Box2I MonoBitmap::NonzeroArea()
{
    Box2I rv;
    int x, y;

    for(x=0;x<m_w;x++)
    {
        for(y=0;y<m_h;y++)
        {
            if(!GetPixel(x,y))
            {
                rv.x=x;
                break;
            }
        }
        if(rv.x)
            break;
    }

    for(x=m_w-1;x>=0;x--)
    {
        for(y=0;y<m_h;y++)
        {
            if(!GetPixel(x,y))
            {
                rv.w=x;
                break;
            }
        }
        if(rv.w)
            break;
    }

    for(y=0;y<m_h;y++)
    {
        for(x=0;x<m_w;x++)
        {
            if(!GetPixel(x,y))
            {
                rv.y=y;
                break;
            }
        }
        if(rv.y)
            break;
    }

    for(y=m_h-1;y>=0;y++)
    {
        for(x=0;x<m_w;x++)
        {
            if(!GetPixel(x,y))
            {
                rv.h=y;
                break;
            }
        }
        if(rv.h)
            break;
    }


    return rv;
}

main()
{
    MonoBitmap *bmp = MonoBitmap::LoadFromFile ( "raster.pcx" );

/*    FILE *f=fopen("raw.data","wb");

    for(int y = 0;y < bmp->Height(); y++)
        for(int x = 0;x < bmp->Width(); x++)
        {
            uint8_t pix = bmp->GetPixel(x,y)? 255: 0;
            fwrite(&pix,1,1,f);
        }


    fclose(f);
*/
    Box2I nz = bmp->NonzeroArea();

    nz.Show();
    delete bmp;
}


