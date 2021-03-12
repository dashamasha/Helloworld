#include <cstdint>            //for int types such as uint32_t
#include "gpioaregisters.hpp" //for Gpioa
#include "gpiocregisters.hpp" //for Gpioc
#include "gpiobregisters.hpp" //for Gpiob
#include "rccregisters.hpp"   //for RCC
#include "tim2registers.hpp"   //for SPI2
#include "nvicregisters.hpp"  //for NVIC
#include "adc1registers.hpp"
#include "adccommonregisters.hpp" 
#include "usart2registers.hpp"
#include <iostream>

extern "C"
{
  constexpr uint32_t BaudRate=9600;
  int __low_level_init(void)
  {
    //config
    RCC::CR::HSEON::On::Set();
    while (!RCC::CR::HSERDY::Ready::IsSet())
    { }
    RCC::CFGR::SW::Hse::Set();
    while (!RCC::CFGR::SWS::Hse::IsSet())
    { }
    RCC::CR::HSION::Off::Set();   
    RCC::AHB1ENR::GPIOAEN::Enable::Set();
    RCC::AHB1ENR::GPIOCEN::Enable::Set();
    GPIOA::MODER::MODER0::Analog::Set();
    GPIOC::MODER::MODER5::Output::Set();
    GPIOA::MODER::MODER5::Output::Set();
    GPIOC::MODER::MODER9::Output::Set();
    GPIOC::MODER::MODER8::Output::Set();
    GPIOA::MODER::MODER2::Alternate::Set(); 
    GPIOA::MODER::MODER3::Alternate::Set();
    GPIOA::OTYPER::OT2::OutputPushPull::Set();
    GPIOA::OTYPER::OT3::OutputPushPull::Set();
    GPIOA::PUPDR::PUPDR2::PullUp::Set();
    GPIOA::PUPDR::PUPDR3::PullUp::Set();
    GPIOA::AFRL::AFRL2::Af7::Set();
    GPIOA::AFRL::AFRL3::Af7::Set();
    
    RCC::APB1ENR::TIM2EN::Enable::Set();//вкл таймер
    TIM2::PSC::Write(7999); //делим на 8к
    TIM2::ARR::Write(500);// скорость подачи
    TIM2::CR1::CEN::Enable::Set(); // включаем счетчик
    
     //UART
    RCC::APB1ENR::USART2EN::Enable::Set();//подача тактирования
    USART2::CR1::M::Data8bits::Set();
    USART2::CR1::PCE::ParityControlDisable::Set();
    USART2::CR1::OVER8::OversamplingBy16::Set();
    USART2::CR2::STOP::Value0::Set(); //1 стоп бит
    //USARTDIV = CLK/(BaudRate*8*(2 - OVER8))?
    USART2::CR1::UE::Enable::Set(); //Включение USART
    USART2::CR1::TE::Enable::Set(); //Включение передачи
    uint32_t USART_DIV = 8'000'000/(BaudRate*8*(2 - USART2::CR1::OVER8::Get()));
    USART2::BRR::DIV_Mantissa::Set(USART_DIV);
    
    
    
    //ADC
//    RCC::APB2ENR::ADC1EN::Enable::Set();                               //Подаем тактирование на АЦП                               
//    ADC_Common::CCR::TSVREFE::Enable::Set();                           //Включаем температурный дачтик 
//    ADC1::CR1::RES::Bits12::Set();                                     //Разрядность 12
//    ADC1::CR1::SCAN::Enable::Set();                                    //Режим сканирования каналов
//   
    
    
//    ADC1::SQR3::SQ1::Channel18::Set(); //Первое измерение на канале температурного датчика
//    ADC1::SQR3::SQ2::Channel17::Set(); //Второе измерение нак канале опорного напряжения
//    ADC1::SQR3::SQ3::Channel0::Set(); //Третье измерение на канале 0
//    ADC1::CR2::CONT::SingleConversion::Set();// Одиночное преобразование
//    ADC1::SQR1::L::Conversions3::Set();// 3 Канала
//    ADC1::CR2::EOCS::After_Each_Chanell::Set(); //Флаг окончания преобразования  АЦП, ставится после преоб каждого канала
//    ADC1::CR2::ADON::Enable::Set(); //Включение АЦП
    return 1;
  }
}
//   float V25 = 0.76F;
//   float AVG_Slope = 0.0025F;
//   float Temp = 0.0F;
//   auto Ts_Call = *reinterpret_cast <uint16_t*> (0x1FFF7A2C);
//   auto Ts_Cal2 = *reinterpret_cast <uint16_t*> (0x1FFF7A2E);
//   auto Vref = *reinterpret_cast <uint16_t*> (0x1FFF7A2A);
int main()
{
  char buff[] = " Hello, world! ";
  
  for(;;)
  {
    for(int i=0; i<strlen(buff); i++)
    {
      USART2::DR::Write(buff[i]);
      while(!USART2::SR::TXE::DataRegisterEmpty::IsSet())
            {
            }
    }
    
  }
  
 
//  for(;;)
//  {
//   ADC1::CR2::SWSTART::On::Set();
//     while(!ADC1::SR::EOC::ConversionComplete::IsSet())
//    {};
//     uint32_t Temp_data =   ADC1::DR::Get();
//      while(!ADC1::SR::EOC::ConversionComplete::IsSet())
//    {};
//    uint32_t Vref_Data =   ADC1::DR::Get();
//      while(!ADC1::SR::EOC::ConversionComplete::IsSet())
//    {};
//     uint32_t Res_Data =   ADC1::DR::Get(); 
//  
//
//  float temp = ((30.0F - 110.0F)*Temp_data + (Ts_Call*110.0F - Ts_Cal2*30.0F))/(Ts_Call-Ts_Cal2);
//  float K = Vref/Vref_Data;
//  float Voltage = (Res_Data * 3.3f/4095.0f) * 1.0f/K;
//  std::cout << " Темература измеренная °С : " <<  temp*K << std::endl;  
//  std::cout << " Опроное напряжение : " <<  Vref_Data << std::endl; 
//  std::cout << " Напряжение : " <<  Voltage << std::endl; 
//  
//  
//  if(temp>29)
//  {
//    GPIOC::BSRR::BS5::High::Write();
//  }
//  else
//  {
//    GPIOC::BSRR::BR5::Low::Write();
//  }
//   if(temp>30)
//  {
//    GPIOC::BSRR::BS8::High::Write();
//  }
//  else
//  {
//    GPIOC::BSRR::BR8::Low::Write();
//  }
//   if(temp>31)
//  {
//    GPIOC::BSRR::BS9::High::Write();
//  }
//  else
//  {
//    GPIOC::BSRR::BR9::Low::Write();
//  }
//   if(temp>32)
//  {
//    GPIOA::BSRR::BS5::High::Write();
//  }
//  else
//  {
//    GPIOA::BSRR::BR5::Low::Write();
//  }
//}
}
