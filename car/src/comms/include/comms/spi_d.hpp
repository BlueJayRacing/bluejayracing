#ifndef SPI_D_HPP
#define SPI_D_HPP

class spi_d {
public:
  int pi;
  int spi_handle;
  
  spi_d(): pi{-1}, spi_handle{-1} {return;}
  spi_d(int pi, int fd):pi{pi}, spi_handle{fd} {return;}
};

#endif