#include "Ublox.h"

Ublox::Tokeniser::Tokeniser(char* _str, char _token)
{
    str = _str;
    token = _token;
}

bool Ublox::Tokeniser::next(char* out, int len)
{
    uint8_t count = 0;

    if(str[0] == 0)
        return false;

    while(true)
    {
        if(str[count] == '\0')
        {
            out[count] = '\0';
            str = &str[count];
            return true;
        }
        if(str[count] == token)
        {
            out[count] = '\0';
            count++;
            str = &str[count];
            return true;
        }

        if(count < len)
            out[count] = str[count];
        count++;
    }
    return false;
}
bool Ublox::encode(char c)
{
    buf[pos] = c;
    pos++;

    if(c == '\n') //linefeed
    {
        bool ret = process_buf();
        memset(buf, '\0', 120);
        pos = 0;
        return ret;
    }

    if(pos >= 120) //avoid a buffer overrun
    {
        memset(buf, '\0', 120);
        pos = 0;
    }
    return false;
}


bool Ublox::process_buf()
{
    if(!check_checksum()) //if checksum is bad
    {
        return false; //return
    }

    //otherwise, what sort of message is it
    if(strncmp(buf, "$GNGGA", 6) == 0)
    {
        read_gga();
    }
    return true;
}

// GNGGA 
void Ublox::read_gga()
{
    int counter = 0;
    char token[20];
    
    Tokeniser tok(buf, ',');

    while(tok.next(token, 20))
    {
        switch(counter)
        {
        case 2: //latitude
        {
            float llat = atof(token);
            int ilat = llat/100;
            double mins = fmod(llat, 100);
            latitude = ilat + (mins/60);
        }
        break;
        case 4: //longitude
        {
            float llong = atof(token);
            int ilat = llong/100;
            double mins = fmod(llong, 100);
            longitude = ilat + (mins/60);
        }     
        break;
        }
        counter++;
    }
}




bool Ublox::check_checksum()
{
    if (buf[strlen(buf)-5] == '*')
    {
        uint16_t sum = parse_hex(buf[strlen(buf)-4]) * 16;
        sum += parse_hex(buf[strlen(buf)-3]);

        for (uint8_t i=1; i < (strlen(buf)-5); i++)
            sum ^= buf[i];
        if (sum != 0)
            return false;

        return true;
    }
    return false;
}


uint8_t Ublox::parse_hex(char c)
{
    if (c < '0')
        return 0;
    if (c <= '9')
        return c - '0';
    if (c < 'A')
        return 0;
    if (c <= 'F')
        return (c - 'A')+10;
    return 0;
}


