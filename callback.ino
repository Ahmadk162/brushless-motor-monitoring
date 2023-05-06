void callback(char* topic, byte* payload, unsigned int length) {

    Serial.println("-------new message from broker-----");
    Serial.print("channel:");
    Serial.println(topic);
    Serial.print("data:");  
    Serial.write(payload, length);
    Serial.println();
  if (strcmp(topic, MQTT_SERIAL_RECEIVER_motor_speed) == 0)
  {
    char command_str1[length];
    Serial.print("\ncommand: ");
    for (int  i = 0; i < length;  i++) {    
      command_str1[i] = (char)payload[i];    
      Serial.println((char)command_str1[i]);
    }  
    speed_of_motor_in = (float)atoi((char*)command_str1);     
    Serial.println(speed_of_motor_in);
        analogWrite(PWM_ch2,120);
    
  }
  
  if (strcmp(topic, MQTT_SERIAL_RECEIVER_kp) == 0)
  {
    char command_str1[length];
    Serial.print("\ncommand: ");
    for (int  i = 0; i < length;  i++) {    
      command_str1[i] = (char)payload[i];    
      Serial.println((char)command_str1[i]);
    }  
    Kp = (float)atoi((char*)command_str1);     
    Serial.println(Kp);
    
  }
    if (strcmp(topic, MQTT_SERIAL_RECEIVER_ki) == 0)
  {
    char command_str1[length];
    Serial.print("\ncommand: ");
    for (int  i = 0; i < length;  i++) {    
      command_str1[i] = (char)payload[i];    
      Serial.println((char)command_str1[i]);
    }  
    Ki = (float)atoi((char*)command_str1);     
    Serial.println(Ki);
    
  }
    if (strcmp(topic, MQTT_SERIAL_RECEIVER_kd) == 0)
  {
    char command_str1[length];
    Serial.print("\ncommand: ");
    for (int  i = 0; i < length;  i++) {    
      command_str1[i] = (char)payload[i];    
      Serial.println((char)command_str1[i]);
    }  
    Kd = (float)atoi((char*)command_str1);     
    Serial.println(Kd);
    
  }
  
  if (strcmp(topic, MQTT_SERIAL_RECEIVER_CH_pwm1) == 0) {
    char command_str1[length];
    Serial.print("\ncommand: ");
    for (int  i = 0; i < length;  i++) {    
      command_str1[i] = (char)payload[i];    
      Serial.print((char)command_str1[i]);
    }  
    pwm_cmd1 = (float)atoi((char*)command_str1);    
    // conversion from max=100% PWM to 8-bits output    
    pwm_cmd1 *= 255.0/100;    
    Serial.print("PWM1 Ratio:");    
    Serial.println(pwm_cmd1);
    if (pwm_cmd1>=0,pwm_cmd1<=255){
      analogWrite(PWM_ch1,pwm_cmd1);
      pwm_percentage_1=pwm_cmd1*100/254;
      dtostrf(pwm_percentage_1, 1, 2, pwm1);
      client.publish(MQTT_SERIAL_PUBLISH_CH_pwm1, pwm1);
    }
  }  
/*
  if (strcmp(topic, MQTT_SERIAL_RECEIVER_CH_pwm2) == 0) {
    char command_str2[length];
    Serial.print("\ncommand: ");
    for (int  i = 0; i < length;  i++) {    
      command_str2[i] = (char)payload[i];    
      //command_str2[i] = (char)payload[i];    
      Serial.print((char)command_str2[i]);
      }  
     pwm_cmd2 = (float)atoi((char*)command_str2);    
     // conversion from max=100% PWM to 8-bits output    
     pwm_cmd2 *= 255.0/100;    
     Serial.print("PWM2 Ratio:");    
     Serial.println(pwm_cmd2); 
     if (pwm_cmd2>=0,pwm_cmd2<=255){
      analogWrite(PWM_ch2,pwm_cmd2);  
      pwm_percentage_2=pwm_cmd2*100/254;
      dtostrf(pwm_percentage_2, 1, 2, pwm2);
      client.publish(MQTT_SERIAL_PUBLISH_CH_pwm2, pwm2);
      }
    }*/
  }
