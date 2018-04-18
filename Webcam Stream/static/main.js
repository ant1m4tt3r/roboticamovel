let STOP_REQUESTS = false;
let currentRequest = null;

$(document).ready(() => {

    $('#commander').off('click').on('click', function () {
        if (checkForStop(this))
            STOP_REQUESTS = true
        else
            STOP_REQUESTS = false

        saveImage(this, false);
    });

});

function saveImage(btn, repeating = false) {
    if (!repeating) blockElement(btn)

    if (STOP_REQUESTS) {
        setElementUnstopable(btn);
        return;
    }

    currentRequest = $.ajax({
        url: '/save_image',
        mehtod: 'POST',
        data: {},
        success: (data) => {
            console.log(data);
            addDataToList(data);

            setTimeout(() => saveImage(btn, true), 500);

            unblockElement(btn);
            if (!STOP_REQUESTS)
                setElementStopable(btn);
        },
        error: (err) => {
            console.error(err);
            unblockElement(btn);
        }
    });
}

function setElementUnstopable(btn) {
    $(btn).attr('stop', false).removeClass('btn-danger').addClass('btn-primary').html('Começar Reconhecimento');
    removeList();
    if (currentRequest) currentRequest.abort();
}

function setElementStopable(btn) {
    $(btn).attr('stop', true).addClass('btn-danger').removeClass('btn-primary').html('Parar');
}

function checkForStop(btn) {
    return $(btn).attr('stop') === "true";
}

function removeList() {
    $('#matches-title').html('');
    $('#matches').html('');
}

function addDataToList(data) {
    if (data) {
        $('#matches-title').html('Possibilidades:');
    } else {
        $('#matches-title').html('Nada encontardo :(');
    }

    try {
        const list = data.map((e) => {
            return `<li>
                    <button type="button" class="btn btn-light">
                        ${e.string} <span class="badge badge-primary">${e.score}</span>
                    </button>
                </li>`
        });

        $('#matches').html(list);
    } catch (err){
        console.log('err');
    }
}

function blockElement(el) {
    $(el).addClass('disabled');
}

function unblockElement(el) {
    $(el).removeClass('disabled');
}