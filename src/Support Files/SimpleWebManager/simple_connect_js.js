const elem_connected = document.getElementsByClassName("con_stat")[0];
const elem_con_ip = document.getElementsByClassName("con_ip")[0];
const elem_ip_p = document.getElementsByClassName("ip_p")[0];
const elem_wifi_form = document.getElementsByClassName("wifi_form")[0];
const elem_wifi_submit_button = document.getElementsByClassName("wifi_submit_button")[0];
const elem_saved_wifi_div = document.getElementsByClassName("saved_wifi_div")[0];
const elem_rem_saved_wifi_button = document.getElementsByClassName("rem_saved_wifi_button")[0];

elem_wifi_submit_button.addEventListener("click", form_submit);
elem_rem_saved_wifi_button.addEventListener("click", rem_saved_wifi);

async function rem_saved_wifi() {
    try {
        const response = await fetch("rem_saved_wifi")
        if (!response.ok) {
            throw new Error(`Response status: ${response.status}`);
        }
    } catch (error) {
        console.error(error.message);
        elem_connected.innerHTML =
        "ESP32 disconnected (or some other remove saved WiFi error)";
    }
}

async function form_submit() {
    try {
        const response = await fetch("simple_connect", {
            method: "POST",
            headers: {
                "Content-Type": "application/x-www-form-urlencoded",
            },
            body: new URLSearchParams(new FormData(elem_wifi_form)),
        });
        if (!response.ok) {
            throw new Error(`Response status: ${response.status}`);
        }
    } catch (error) {
        console.error(error.message);
        elem_connected.innerHTML =
        "ESP32 disconnected (or some other form submit error)";
    }
}

async function check_connect() {
    try {
        const response = await fetch("simple_check_connect");
        if (!response.ok) {
            throw new Error(`Response status: ${response.status}`);
        }
        let json = await response.json();
        console.log(json);
        switch (json["con_stat"]) {
            case "0":
                elem_connected.innerHTML = "WiFi connected";
                elem_con_ip.innerHTML = json["ip"]
                elem_ip_p.removeAttribute("hidden");
                break;
            case "1":
                elem_connected.innerHTML = "attempted WiFi connection failed";
                elem_ip_p.setAttribute("hidden", "hidden");
                break;
            case "2":
                elem_connected.innerHTML = "WiFi disconnected";
                elem_ip_p.setAttribute("hidden", "hidden");
                break;
        }
        if(json["saved"]=="1"){
            elem_saved_wifi_div.removeAttribute("hidden");
        }
        else{
            elem_saved_wifi_div.setAttribute("hidden", "hidden");
        }
    } catch (error) {
        console.error(error.message);
        elem_connected.innerHTML = "ESP32 disconnected";
    }
}

setInterval(check_connect, 250);
console.log("Interval enabled.");