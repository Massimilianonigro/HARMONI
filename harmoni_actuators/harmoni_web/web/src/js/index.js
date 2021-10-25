var page = "pageContent";




$(document).ready(function () {

    $.getJSON("src/config/config.json", function (data) {
        $.each(data, function (key, val) {
            if (page == key) {
                $.each(val, function (k, v) {
                    console.log(v);
                    var id = v.id;
                    var component = v.component;
                    var children = v.children;
                    var id_parent = "body_page";
                    var comp_class = "";
                    if(v.hasOwnProperty('comp_class')){
                        comp_class = v.comp_class
                    }
                    handleComponents(children, id, component, id_parent, comp_class);
                    $("#" + id).hide();
                });
            };
        });
    })
        .done(function () {
            if(view.includes("code")){
                console.log("Added code script")
                addCode()
                console.log("Added record script")
                record()
            }
            if(show){
                $("#"+view).show();
            }
            // ADD HERE the event click event
            $("button").on("click", function () {
                var value = document.getElementById("input_1")
                //var value_item = $(this).closest("div.button").find("input[name='inputext']").val();
                if(view.includes("code")){
                    var value_item = $(this).closest('container').find('div.button');
                    console.log(value_item)
                    var value = value_item.prevObject.prevObject[0].previousSibling
                } 
                setValueButton(this, value);
                clickListener(this);
            });
            $("a").on("click", function () {
                clickListener(this);
            });
        });
});


function viewListener(view) {
    console.log("VIEW LISTENER")
    //Waiting for the view request from the ROS package
    console.log(view.data)
    var data = view.data.replace(/'/g, '"')
    var json_data = JSON.parse(data)
    var component = json_data.component_id
    var content = json_data.set_content
    //console.log(content)
    
    if (content != "") {
        if (component.includes("img")) {
            displayShow(component)
            $("#" + component).attr("src", content);
            $("#" + component).attr("value", content);
            $('img', "#"+component).attr('src', content);
            $("#"+ component).children().unbind('click');
        }
        else if (component.includes("txt")) {
            displayShow(component)
            if (content.length > 10){
                $("#" + component)[0].style.fontSize = "400%" ;
            }
            else {
                $("#" + component)[0].style.fontSize = "1000%" ;
            }
            //console.log($("#" + component)[0]) ;
            //$("#" + component)[0].style.display = 'inline-block';            
            //$("#" + component)[0].style.visibility = 'visible';
            $("#" + component).html(content)
        }
        else {
            displayShow(component)
            $("#" + component).html(content)
        }
        $("#" + component).show();

    } else if (component.includes("container")) {
        $(".container").hide()
        displayShow(component)
        //displayHide(component)
    } else if (component.includes("img")){
        //console.log($("#" + component).parent().parent()[0])
        displayHide(component)
       
    } else if (component.includes("txt")){
        displayHide(component)
    }
    disableOptions()
    /*
    if ($("#" + component).find(".button_try").prop("disabled")){
        //console.log("disabling")
        $("#" + component).find(".button_try").prop("disabled", false);
    }
    else{
        //console.log("enabling")
        $("#" + component).find(".button_try").prop("disabled", true);
    }
    if ($("#" + component).find(".card-img-top").prop("disabled")){
        //console.log("disabling")
        $("#" + component).find(".card-img-top").prop("disabled", false);
    }
    else{
        //console.log("enabling")
        $("#" + component).find(".card-img-top").prop("disabled", true);
    }*/
    
    //setTimeout(function(){ $("#"+ component).children().bind('click'); }, 3000);
};


function requestListener(view) {
    console.log("REQUEST LISTENER")
    //Waiting for the view request from the ROS package
    //console.log(view.data)
    var data = view.data.replace(/'/g, '"')
    var json_data = JSON.parse(data)
    var component = json_data.component_id
    var content = json_data.set_content
    //console.log(content)
    if (content != "") {
        if (component.includes("img")) {
            $("#" + component).attr("src", content);
            $("#" + component).attr("value", content);
            $('img', "#"+component).attr('src', content);
            //$("#"+ component).children().bind('click');
        }
        else {
            
            $("#" + component).html(content)
        }
    } else if (component.includes("container")) {
        //$(".container").hide()
    } else if (component.includes("img")){
        //console.log($("#" + component).parentElement.parentElement)
        //$("#" + component).parentElement.parentElement.parentElement.style.display = 'none';
    }
    $("#"+ component).children().bind('click'); 
    enableOptions()
    //$("#" + component).find(".option_choice").prop("disabled", false);
    //$("#" + component).show();
    //setTimeout(function(){ $("#"+ component).children().bind('click'); enableOptions()}, 3000);
};

function setValueButton(clicked_button, value_item){
    //console.log(value_item)
    var selected_butt = clicked_button.id;
    //console.log(selected_butt)
    if(value_item!=null && value_item!=""){
        if(selected_butt!="start_button"){
            var input_value = document.getElementById(value_item.id).value;
            //$("#"+value_item).attr("value")
            $("#"+selected_butt).attr("value",input_value);
        }
    }
}

function clickListener(clicked_component) {
    var selected_item = clicked_component.id;
    var selected_item_id = selected_item
    //$("#"+selected_item).css("opacity", "0.5");
    if (selected_item.includes("img")){
        var selected_item = clicked_component.value;
        var selected_item_id = clicked_component.id;
    }
    console.log("Clicked")
    console.log(clicked_component)
    //$("#"+selected_item).css("opacity", "1");
    if(clicked_component.getAttribute("value")!="" && clicked_component.getAttribute("value")!=null){
        var body =  {component_id:selected_item_id , set_view:clicked_component.getAttribute("value")}
    }else{
        var body =  {component_id:selected_item_id , set_view:""}
    }
    console.log("The response is", body)
    user_response_publisher.publish({data: JSON.stringify(body)})
    disableOptions()       
    // Send the event clicked to the ROS package
}


function disableOptions(){
    $(".option_choice").addClass("disabled")
}

function enableOptions(){
    if ($(".option_choice").hasClass("disabled")){
        $(".option_choice").removeClass("disabled")
    }
}

function displayHide(id_div){
    $("#"+id_div+"").addClass("hidden")
}

function displayShow(id_div){
    if ($("#"+id_div+"").hasClass("hidden")){
        $("#"+id_div+"").removeClass("hidden")
    }
}


function handleComponents(children, id, component, id_parent, comp_class) {
    if (Array.isArray(children)) {
        var component_html = createComponent(component, children, id, comp_class);
        $("#" + id_parent).append(component_html);
        $.each(children, function (k_c, v_c) {
            console.log(v_c);
            var id_c = v_c.id;
            var component_c = v_c.component;
            var children_c = v_c.children;
            var class_c = "";
            if(v_c.hasOwnProperty('comp_class')){
                class_c = v_c.comp_class
            }
            handleComponents(children_c, id_c, component_c, id, class_c);
        });
    } else {
        var component_html = createComponent(component, children, id, comp_class);
        $("#" + id_parent).append(component_html);


    }
}


function createComponent(component, content, id, comp_class) {
    if(comp_class==""){
        comp_class="default"
    }
    switch (component) {
        case "container":
            var html = "<div class ='container "+comp_class+"' id=" + id + "></div>";
            break;
        case "card":
            var html = "<div class=\"col "+comp_class+"\" id=" + id + "><div class=\"card box-shadow\"><a id= " + id +"_a'><img data-src=\"holder.js/100px225?theme=thumb&bg=55595c&fg=eceeef&text=Thumbnail\" alt=\"Thumbnail [100%x225]\" style=\"height: 275px; width: 100%; object-fit: cover;\" data-holder-rendered=\"true\" class=\"card-img-top option_choice\" src=\"" + content + "\"><div class=\"card-body\"><button id= txt_" +id.substring(4)+ " class=\"card-text button_try\"></p></div></a></div></div>"
            break;
        case "card_text":
            var html = "<div class=\"col "+comp_class+"\"><div class=\"card box-shadow\"><a id= " + id + "><div class=\"card-body\"><button id= QA_text_" +id.charAt(4)+ " class=\"card-text button_try\"></p></div></a></div></div>"
            break;
        case "fixed_card_img":
            var html = "<div class=\"col "+comp_class+"\" ><div class=\"card box-shadow\"><a id= " + id + "><img data-src=\"holder.js/100px225?theme=thumb&bg=55595c&fg=eceeef&text=Thumbnail\" alt=\"Thumbnail [100%x225]\" style=\"height: 275px; width: 100%;  object-fit: cover;\" data-holder-rendered=\"true\" class=\"card-img-top option_choice\" src=\"" + content + "\"></p></div></a></div></div>"
            break;
        case "fixed_card_text":
            var html = "<div class=\"col "+comp_class+"\" ><div class=\"card box-shadow\"><a id= " + id + "_a><div class=\"card-body\"><button id= " +id+ " class=\"card-text button_try \" style=\"font-size:1000%\"></p></div></a></div></div>"
            break;
        case "click_img":
            var html = "<a id=" + id +"><img class="+comp_class+"  src=" + content + "></a>";
        case "img":
            var html = "<img class="+comp_class+"  src='" + content + "' id=" + id + " style=\"height: 20%; ; width: 20%;\">";
            break;
        case "video":
            var html = "<video class="+comp_class+"  src='" + content + "' id=" + id + " muted autoplay loop>";
            break;
        case "text":
            if (!Array.isArray(content)) {
                var html = "<p class="+comp_class+"  id=" + id + ">" + content + "</p>";
            } else {
                var html = "<p class="+comp_class+"  id=" + id + "></p>";
            }
            break;
        case "title":
            if (!Array.isArray(content)) {
                var html = "<div class='title' style=\"font-size:500%\" id=" + id + ">" + content + "</div>";
            } else {
                var html = "<div class='title' style=\"font-size:500%\" id=" + id + "></div>";
            }
            break;
        case "input_text":
                var html = "<input class="+comp_class+"  id="+ id +" type='text' name='inputext' >";
                break;
        case "input_number":
            var html = "<input class="+comp_class+"  id="+ id +" type='number' name='inpnumb' maxlength="+content+">";
            break;
        case "button":
            if (!Array.isArray(content)) {
                var html = "<button class="+comp_class+"  id=" + id + " value= ''>" + content + "</button>";
            } else {
                var html = "<button class="+comp_class+"  id=" + id + " value= ''></button>";
            }    
        
            break;
        case "row":
            var html = "<div class='row "+comp_class+"' id=" + id + "></div>";
            break;
        case "col":
            var html = "<div class='col "+comp_class+"' id=" + id + "></div>";
            break;
    }
    return html;
}
