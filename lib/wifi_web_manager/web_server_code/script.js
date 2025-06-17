var last = "";
var failed = false;

function on_update() {
    if (last != "") {
        document.getElementById(last).setAttribute("hidden", "hidden");
        document.getElementById(last).setAttribute("disabled", "disabled");
    }
    let selected = document.getElementById("select").selectedOptions.item(0);
    switch (selected.dataset.auth) {
        case "0":
            last = "open";
            break;
        case "1":
            last = "psk";
            break;
        case "2":
            last = "enterprise";
            break;
        default:
            return;
    }
    document.getElementById(last).removeAttribute("hidden");
    document.getElementById(last).removeAttribute("disabled");
}

function on_submit() {
    failed = false;
}

async function refresh_wifi_list() {
    let url = "fetch";
    let activeTextarea = document.activeElement;
    try {
        const response = await fetch(url);
        if (!response.ok) {
            throw new Error(`Response status: ${response.status}`);
        }
        const json = await response.json();
        let options = "<option value=\"\">--Please choose an option--<\/option>";
        if (!(json.hasOwnProperty(document.getElementById("select").value))) {
            options = "<option selected value=\"\">--Please choose an option--<\/option>";
        }
        Object.entries(json).forEach(([key, value]) => {
            if (document.getElementById("select").value === key) {
                options += "<option selected data-auth=\"" + value.toString() + "\">" + key.toString() + "</option>\n";
            } else {
                options += "<option data-auth=\"" + value.toString() + "\">" + key.toString() + "</option>\n";
            }
        });
        document.getElementById("select").innerHTML = options;
        on_update();
    } catch (error) {
        console.error(error.message);
    }
    if (activeTextarea.id === "username" || activeTextarea.id === "password" || activeTextarea.id === "certificate") {
        activeTextarea.focus();
    }
}

async function refresh_con_status() {
    url = "connected";
    try {
        const response = await fetch(url);
        if (!response.ok) {
            throw new Error(`Response status: ${response.status}`);
        }
        const json = await response.json();
        if (!failed) {
            if (json["connected"] === 0) {
                document.getElementById("con_status").innerHTML = "WiFi Status: not connected.";
            } else if (json["connected"] === 1) {
                document.getElementById("con_status").innerHTML = "WiFi Status: connected.";
            } else if (json["connected"] === -1) {
                document.getElementById("con_status").innerHTML = "WiFi Status: connection error.";
                failed = true;
            }
        }
    } catch (error) {
        console.error(error.message);
        if (error instanceof TypeError) {
            document.getElementById("con_status").innerHTML = "WiFi Status: ESP32 disconnected.";
        }
    }
}

document.getElementById("select").addEventListener("change", on_update);
document.getElementById("form").addEventListener("submit", on_submit);
refresh_wifi_list();
setInterval(refresh_wifi_list, 250);
setInterval(refresh_con_status, 250);