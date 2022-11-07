paper.install(window);

$(function() {
    // Setup directly from canvas id:
    paper.setup('face');

    $('#face').attr({
        width: $('#face').width(),
        height: $('#face').height()
    });

    face = new Face(me);

    $('#toggle-muscles').click(function() {
        face.showMuscles = !face.showMuscles;
    });

    view.draw();

    view.onFrame = function(event) {
        face.loop(event);
    };

    console.log("Setting up sliders");
    $('.slider').each(function() {
        $(this).slider({
            from: 0,
            to: 100,
            step: 1,
            skin: 'round',
            dimension: '&nbsp;%',
            onstatechange: function(newVal) {
                var input = this.inputNode;
                var musclePulls, emotions = {};

                if ($('#emotions').is(':visible') && input.hasClass('emotion')) {
                    $.each(face.recipes, function(emotion, muscles) {
                        emotions[emotion] = $('input#' + emotion).val() / 100;
                    });
                    musclePulls = face.recalculateMusclesFromEmotions(emotions);
                    $.each(musclePulls, function(muscle, pull) {
                        $('input#' + muscle).val(pull * 100);
                    });
                }

                // Move info from muscle sliders into facial simulators
                $.each(face.muscles, function(muscle, pull) {
                    face.muscles[muscle] = $('input#' + muscle).val() / 100;
                });
            }
        });
    });

    $('.muscle-controls').click(function() {
        $('.slider.muscle').each(function() {
            $(this).slider('prc', $(this).val());
        });
    });
    getEmotionKey();

});

function getEmotionKey() {
    var searchStrings = window.location.search.replace('?','').split('&');
    $.each(searchStrings, function(i, string) {
        query = string.split('=');
        switch (query[0]) {
            case 'emotions':
                setTimeout(function() {
                    $.each(query[1].split(''), function(i, val) {
                        var pct = remap(parseInt(val, 16), 0, 15, 0, 100);
                        $('.slider.emotion').eq(i).slider("prc",pct);
                    });
                }, 2000);
                break;
            case 'muscles':
                setTimeout(function() {
                    $.each(query[1].split(''), function(i, val) {
                        var pct = remap(parseInt(val,16), 0, 15, 0, 100);
                        $('slider.muscle').eq(i).slider("prc",pct);
                    });
                }, 2000);
                break;
            default:
                setTimeout(function() {
                    $('input#joy').slider('prc',100);
                }, 2000);
                break;
        }
    });
}
