const elem_connected = document.getElementsByClassName("con_stat")[0];
const elem_wifi_form = document.getElementsByClassName("wifi_form")[0];
const elem_wifi_submit_button =
    document.getElementsByClassName("wifi_submit_button")[0];

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
        let text = await response.text();
        console.log(text);
        switch (text) {
            case "0":
                elem_connected.innerHTML = "WiFi connected";
                break;
            case "1":
                elem_connected.innerHTML = "attempted WiFi connection failed";
                break;
            case "2":
                elem_connected.innerHTML = "WiFi disconnected";
                break;
        }
    } catch {
        elem_connected.innerHTML = "ESP32 disconnected";
    }
}
