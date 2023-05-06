void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    //client.setServer(mqtt_server, mqtt_port);
    if (client.connect(clientId.c_str(),MQTT_USER,MQTT_PASSWORD)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      client.publish("/icircuit/presence/ESP32/", "hello world");
      // ... and resubscribe
      client.subscribe(MQTT_SERIAL_RECEIVER_CH);
      client.subscribe(MQTT_SERIAL_RECEIVER_CH_pwm1);
      client.subscribe(MQTT_SERIAL_RECEIVER_CH_pwm2);
      client.subscribe(MQTT_SERIAL_RECEIVER_motor_speed);
      client.subscribe(MQTT_SERIAL_RECEIVER_kp);
      client.subscribe(MQTT_SERIAL_RECEIVER_ki);
      client.subscribe(MQTT_SERIAL_RECEIVER_kd);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
 }
