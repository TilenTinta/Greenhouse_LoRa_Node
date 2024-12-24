function decodeUplink(input) {
    const bytes = input.bytes;
  
    if (bytes.length < 14) {
      return {
        data: {},
        warnings: [],
        errors: ["Invalid payload length"]
      };
    }
  
    const decodedData = {
      device_id: bytes[0],
      error_flag: bytes[1],
      error_send_count: bytes[2],
      battery_voltage: bytes[3] / 10.0,
      air_temperature: (bytes[4] | (bytes[5] << 8) | (bytes[6] << 16) | (bytes[7] << 24)) / 100.0, // assuming temperature in centigrade scaled by 100
      air_humidity: bytes[8],
      air_pressure: (bytes[9] | (bytes[10] << 8) | (bytes[11] << 16) | (bytes[12] << 24)),
      earth_humidity: bytes[13],
    };
  
    return {
      data: decodedData,
      warnings: [],
      errors: []
    };
  }
  