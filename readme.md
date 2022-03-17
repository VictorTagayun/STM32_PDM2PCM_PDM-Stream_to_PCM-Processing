# Study OF PDM to PCM conversion  

## Purpose of Project

Learn how to:
1. Interface to a PDM microphone
2. Process PDM data to PCM

This project is preceded by interfacing the PDM microphone to an SPI interface of an MCU as discussed here https://github.com/VictorTagayun/STM32_SPI-TX-RX-DMA_SingleBoard

This project will:

1. Use a predefined stream of PDM data and convert/pack to byte size for input to the PDM filter
2. Feed the byte converted PDM data to PDM Filter and check the PCM-converted data and plot to excel/speadsheet

## Steps / Procedure

Get PDM bitstream data from 

1. Single cycle Full scale sinewave https://github.com/siorpaes/pdm_playground/blob/9aa442d8a7e3265fbcdbfb66111b8dbad301e928/pdmgenerator/Host/tones/pdm1MHz/tone500Hz.h
2. Single cycle Lower amplitude sinewave https://github.com/siorpaes/pdm_playground/blob/9aa442d8a7e3265fbcdbfb66111b8dbad301e928/pdmgenerator/Host/tones/pdm1MHz/tone500HzA4.h

Since these data are 2000 bits only, it is not divisible by a factor of 64 which will be chosen as the decimator qty/factor. To remedy this, we need to multiply the PDM data by 4. So we get 8000bit which will yield exatly 125 PCM data.

See details of the problem here.

https://community.st.com/s/question/0D53W000019CZzmSAG/pdmfilter-erroneous-1st-2-pcmconverted-data

and the explanation here

https://github.com/siorpaes/pdm_playground/issues/1#issuecomment-946585626

### Still a work in progress


## Links related to this github


## Other Links

[Go to Main Github IO page](https://victortagayun.github.io/)

[Go to Github repositories](https://github.com/VictorTagayun?tab=repositories)

[Go to Github page](https://github.com/VictorTagayun)

[Go to My Linkedin](https://www.linkedin.com/in/victortagayun/)

[Go to My Linkedin posts/shares/activities](https://www.linkedin.com/in/victortagayun/detail/recent-activity/shares/)

*Disclaimer:*
[Updated Disclaimer](https://github.com/VictorTagayun/GlobalDisclaimer)


*The projects posted here are for my Personal reference, learning and educational purposes only.*
*The purpose of a certain project may be for testing a module and may be just a part of a whole project.*
*It should not be used in a production or commercial environment.*
*Any cause of injury and/or death is the sole responsibility of the user.*
