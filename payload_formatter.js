function decodeUplink(input) {
  var decoded = {};

  decoded.latitude = ((input.bytes[0]<<16)>>>0) + ((input.bytes[1]<<8)>>>0) + input.bytes[2];
  decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;

  decoded.longitude = ((input.bytes[3]<<16)>>>0) + ((input.bytes[4]<<8)>>>0) + input.bytes[5];
  decoded.longitude = (decoded.longitude / 16777215.0 * 360) - 180;

  var speedValue = (((input.bytes[6]<<8)>>>0) + input.bytes[7])/10;
  decoded.speed = speedValue;

  decoded.direction = input.bytes[8] / 10.0;
    
    return {
        data: {
            latitude: decoded.latitude,
            longitude: decoded.longitude,
            altitude: decoded.speed,
            direction: decoded.direction
        },
        warnings: [],
        errors: []
    };
}
