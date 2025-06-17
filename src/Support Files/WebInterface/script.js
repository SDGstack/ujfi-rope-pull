// DOM objects for abs movement
// Encompassing div
const elem_div_home_found = document.getElementsByClassName("home_found")[0];
const elem_p_home_not_found =
    document.getElementsByClassName("home_not_found")[0];

// Buttons
const elem_submit_abs = document.getElementsByClassName("abs_submit_button")[0];
const elem_abs_unit = document.getElementsByClassName("abs_unit")[0];
const elem_toggle_sign_abs =
    document.getElementsByClassName("toggle_sign_abs")[0];
// Angle input, select
const elem_angle_abs = document.getElementsByClassName("angle_abs")[0];
const elem_angle_abs_input = document.getElementsByClassName("abs_val_input")[0];
const elem_angle_abs_input_set = document.getElementsByClassName("set_abs")[0];
// Events for abs
elem_angle_abs_input_set.addEventListener("click", click_set_abs_input);
elem_toggle_sign_abs.addEventListener("click", click_toggle_sign_abs);
elem_submit_abs.addEventListener("click", () => click_submit_abs(), false);
elem_abs_unit.addEventListener("change", select_unit_abs);

// DOM objects for rel movement
// Buttons
const elem_submit_rel = document.getElementsByClassName("rel_submit_button")[0];
const elem_toggle_sign_rel =
    document.getElementsByClassName("toggle_sign_rel")[0];
const elem_rel_unit = document.getElementsByClassName("rel_unit")[0];
// Angle input, select
const elem_angle_rel = document.getElementsByClassName("angle_rel")[0];
const elem_angle_rel_input = document.getElementsByClassName("rel_val_input")[0];
const elem_angle_rel_input_set = document.getElementsByClassName("set_rel")[0];
// Events for rel
elem_angle_rel_input_set.addEventListener("click", click_set_rel_input);
elem_toggle_sign_rel.addEventListener("click", click_toggle_sign_rel);
elem_submit_rel.addEventListener("click", () => click_submit_rel(), false);
elem_rel_unit.addEventListener("change", select_unit_rel);


// DOM object for current angle
const elem_angle_cur = document.getElementsByClassName("angle_cur")[0];

// DOM object for find home button
const elem_find_home = document.getElementsByClassName("submit_find_home")[0];
// Event for find home button
elem_find_home.addEventListener("click", () => click_submit_find_home(), false);

// DOM object for connection status
const elem_con_stat = document.getElementsByClassName("con_stat")[0];

// DOM object for value unit
var disp_coef_deg_per_unit = 1.0;
const elem_select_unit = document.getElementsByClassName("select_unit")[0];
const elem_unit_text = document.getElementsByClassName("unit");
const elem_current_mm_per_rot = document.getElementsByClassName("esp_val")[0];
const elem_set_mm_per_rot =
    document.getElementsByClassName("set_mm_per_rot")[0];
const elem_submit_set_mm_per_rot = document.getElementsByClassName(
    "submit_set_mm_per_rot"
)[0];
var current_mm_per_rot = NaN;
// Events for value unit DOM objects
elem_select_unit.addEventListener("change", select_unit_change);
elem_submit_set_mm_per_rot.addEventListener("click", click_submit_mm_per_rot);

// Variable for abs angle
var abs_angle = parseInt(elem_angle_cur.innerHTML) * disp_coef_deg_per_unit;
// Variable for rel angle
var rel_angle = parseInt(elem_angle_cur.innerHTML) * disp_coef_deg_per_unit;

function select_unit_abs() {
    if (isNaN(current_mm_per_rot) && elem_abs_unit.value == "4") {
        elem_angle_abs_input_set.setAttribute("disabled", "true");
    } else {
        elem_angle_abs_input_set.removeAttribute("disabled");
    }
}

function select_unit_rel() {
    if (isNaN(current_mm_per_rot) && elem_rel_unit.value == "4") {
        elem_angle_rel_input_set.setAttribute("disabled", "true");
    } else {
        elem_angle_rel_input_set.removeAttribute("disabled");
    }
}

function select_unit_change() {
    let unit_text = "";
    switch (elem_select_unit.value) {
        case "1":
            disp_coef_deg_per_unit = 1.0;
            unit_text = "deg";
            break;
        case "2":
            disp_coef_deg_per_unit = 360.0;
            unit_text = "rot";
            break;
        case "3":
            disp_coef_deg_per_unit = 360.0 / (2.0 * Math.PI);
            unit_text = "rad";
            break;
        case "4":
            disp_coef_deg_per_unit = 360.0 / current_mm_per_rot;
            unit_text = "mm";
            break;
    }
    for (var i = 0; i < elem_unit_text.length; i++) {
        elem_unit_text[i].innerHTML = unit_text;
    }
    refresh_unit_vals();
}

function refresh_unit_vals() {
    elem_angle_abs.innerHTML = (abs_angle / disp_coef_deg_per_unit).toFixed(2);
    elem_angle_rel.innerHTML = (rel_angle / disp_coef_deg_per_unit).toFixed(2);
}

function click_set_abs_input() {
    let local_unit_mult = disp_coef_deg_per_unit;
    if (isNaN(disp_coef_deg_per_unit)) { // Do not modify set value if display units multiplier is nan (if mm_per_rot==nan)
        return;
    }
    switch (elem_abs_unit.value) {
        case "1":
            local_unit_mult = 1.0;
            break;
        case "2":
            local_unit_mult = 360.0;
            break;
        case "3":
            local_unit_mult = 360.0 / (2.0 * Math.PI);
            break;
        case "4":
            if (isNaN(current_mm_per_rot)) {
                return;
            }
            local_unit_mult = 360.0 / current_mm_per_rot;
            break;
    }
    abs_angle = parseInt(elem_angle_abs_input.value) * local_unit_mult;
    refresh_unit_vals();
}

function click_set_rel_input() {
    let local_unit_mult = disp_coef_deg_per_unit;
    if (isNaN(disp_coef_deg_per_unit)) { // Do not modify set value if display units multiplier is nan (if mm_per_rot==nan)
        return;
    }
    switch (elem_rel_unit.value) {
        case "1":
            local_unit_mult = 1.0;
            break;
        case "2":
            local_unit_mult = 360.0;
            break;
            case "3":
                local_unit_mult = 360.0 / (2.0 * Math.PI);
            break;
        case "4":
            if (isNaN(current_mm_per_rot)) {
                return;
            }
            local_unit_mult = 360.0 / current_mm_per_rot;
            break;
    }
    rel_angle = parseInt(elem_angle_rel_input.value) * local_unit_mult;
    refresh_unit_vals();
}

function click_toggle_sign_abs() {
    abs_angle *= -1;
    refresh_unit_vals();
}

function click_toggle_sign_rel() {
    rel_angle *= -1;
    refresh_unit_vals();
}

async function click_submit_mm_per_rot() {
    const response = await fetch("/set_mm_per_rot", {
        method: "POST",
        headers: {
            "Content-Type": "application/x-www-form-urlencoded",
        },
        body: new URLSearchParams({ mm_per_rot: elem_set_mm_per_rot.value }),
    });
}

async function click_submit_find_home() {
    const response = await fetch("/find_home");
}

async function click_submit_rel() {
    const response = await fetch("/by_angle", {
        method: "POST",
        headers: {
            "Content-Type": "application/x-www-form-urlencoded",
        },
        body: new URLSearchParams({ angle_val: rel_angle }),
    });
}

async function click_submit_abs() {
    const response = await fetch("/set_angle", {
        method: "POST",
        headers: {
            "Content-Type": "application/x-www-form-urlencoded",
        },
        body: new URLSearchParams({ angle_val: abs_angle }),
    });
}

async function refresh_angle() {
    url = "fetch_angle";
    try {
        const response = await fetch(url);
        if (!response.ok) {
            throw new Error(`Response status: ${response.status}`);
        }
        const json = await response.json();
        let angle = parseFloat(json["angle_val"]);
        current_mm_per_rot = parseFloat(json["mm_per_rot"]);
        elem_current_mm_per_rot.innerHTML = current_mm_per_rot;
        if (!isNaN(angle)) {
            elem_angle_cur.innerHTML = angle/disp_coef_deg_per_unit;
            elem_div_home_found.removeAttribute("hidden");
            elem_p_home_not_found.setAttribute("hidden", "hidden");
        } else {
            elem_p_home_not_found.removeAttribute("hidden");
            elem_div_home_found.setAttribute("hidden", "hidden");
        }
        elem_submit_set_mm_per_rot.removeAttribute("hidden");
        elem_set_mm_per_rot.removeAttribute("disabled");
        elem_con_stat.innerHTML = "ESP32 connected";
        select_unit_change()
    } catch (error) {
        console.error("Angle fetch error: " + error.message);
        elem_con_stat.innerHTML = "ESP32 disconnected.";
        current_mm_per_rot = NaN;
        elem_current_mm_per_rot.innerHTML = current_mm_per_rot;
        elem_submit_set_mm_per_rot.setAttribute("hidden", "hidden");
        elem_set_mm_per_rot.setAttribute("disabled", "true");
        select_unit_change()
    }
}

// Refresh interval for current angle
setInterval(refresh_angle, 250);
console.log("Interval enabled.");
