#pragma once
#include "xNetInterface.h"

// A wired interface: any iEthernetDevice, whatever bus it is on.
//
//     W5500::w5500_param_t param;                     // W5500 over SPI
//     W5500::w5500<Stm32HalSpiBlockTransport> chip(param, &hspi1,
//         W5500_CS_GPIO_Port, W5500_CS_Pin, spi1Mutex);
//     xEthernet eth(chip);
//
//     eth.begin(netConfig);       // then start the driver thread: eth.run()
//     eth.waitLinkUp(5000);
//     xClient client(eth);        // see xClient.h
//
// Everything is in xNetInterface; this class is the type you hold for
// a wired interface, as xWifi is for a wireless one.
class xEthernet : public xNetInterface {
public:
    explicit xEthernet(iEthernetDevice& dev, const Config& cfg = Config())
        : xNetInterface(dev, cfg) {}
};
