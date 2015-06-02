

function toggle_topic(data_location){
    if ($("#" + data_location).is(":hidden")){
	$("#" + data_location).slideDown();
    } else {
	$("#" + data_location).slideUp();
    }
    
};

function dialog_close(event, ui){

    if (!this.closed){
	$(this).hide("slide", {direction:'up'});
	this.closed = true;
    } else {
	$(this).show("slide", {direction:'up'});
	this.closed = false;
    }
    return false;
}

