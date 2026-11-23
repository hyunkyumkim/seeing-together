(function () {
  var btn = document.getElementById("exploreBtn");
  var menu = document.getElementById("exploreMenu");
  if (!btn || !menu) return;

  function close() {
    menu.hidden = true;
    btn.setAttribute("aria-expanded", "false");
  }

  btn.addEventListener("click", function (event) {
    event.stopPropagation();
    var open = menu.hidden;
    menu.hidden = !open;
    btn.setAttribute("aria-expanded", open ? "true" : "false");
  });

  menu.addEventListener("click", function (event) {
    if (event.target.closest("a")) close();
  });

  document.addEventListener("click", function (event) {
    if (!menu.hidden && !btn.contains(event.target)) close();
  });

  document.addEventListener("keydown", function (event) {
    if (event.key === "Escape") close();
  });
})();
