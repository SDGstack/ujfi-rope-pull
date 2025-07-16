const elem_connected = document.getElementsByClassName("con_stat")[0];
const elem_con_ip = document.getElementsByClassName("con_ip")[0];
const elem_ip_p = document.getElementsByClassName("ip_p")[0];
const elem_wifi_form = document.getElementsByClassName("wifi_form")[0];
const elem_wifi_submit_button = document.getElementsByClassName("wifi_submit_button")[0];

elem_wifi_submit_button.addEventListener("click", form_submit);

setInterval(check_connect, 250);

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
    } catch {
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
        console.log(text);
        switch (json["con_stat"]) {
            case "0":
                elem_connected.innerHTML = "WiFi connected";
                elem_con_ip.innerHTML = json["ip"]
                elem_ip_p.removeAttribute("hidden");
                break;
            case "1":
                elem_connected.innerHTML = "attempted WiFi connection failed";
                elem_ip_p.setAttribute("hidden");
                break;
            case "2":
                elem_connected.innerHTML = "WiFi disconnected";
                elem_ip_p.setAttribute("hidden");
                break;
        }
    } catch {
        elem_connected.innerHTML = "ESP32 disconnected";
    }
}
