  function colours(index) 
  {
    switch(index)
    {
      case 0:
        $('.colour1').css('background', '#336699');
        $('.colour2').css('background', '#666666');
        $('.colour3').css('background', '#999999');
        $('.colour4').css('background', '#BBBBBB');
        $('.colourTxt').css('color', '#FFF');
        $('.colourTxtH').css('color', '#FFF');
        break;
      case 1:
        $('.colour1').css('background', '#222222');
        $('.colour2').css('background', '#FFFFFF');
        $('.colour3').css('background', '#DDDDDD');
        $('.colour4').css('background', '#EEEEEE');
        $('.colourTxt').css('color', '#000');
        $('.colourTxtH').css('color', '#FFF');
        break;
      case 2:
        $('.colour1').css('background', '#443266');
        $('.colour2').css('background', '#666666');
        $('.colour3').css('background', '#C3C3E5');
        $('.colour4').css('background', '#F1F0FF');
        $('.colourTxt').css('color', '#000');
        $('.colourTxtH').css('color', '#FFF');
        break;
      default:
        break;
    }
  }