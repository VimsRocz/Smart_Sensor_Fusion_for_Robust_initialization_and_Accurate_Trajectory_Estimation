#!/usr/bin/env python3
"""Tkinter GUI for batch-capable Task 1-7 GNSS/IMU fusion runs."""

from __future__ import annotations

import itertools
import math
import os
import queue
import shutil
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

REPO_ROOT = Path(__file__).resolve().parent
RUNNER = REPO_ROOT / "PYTHON" / "main.py"
RELEASE_RUNNER = REPO_ROOT / "PYTHON" / "run_release.py"
METHODS = (
    "TRIAD",
    "Davenport",
    "SVD",
    "TRIAD,Davenport",
    "TRIAD,SVD",
    "Davenport,SVD",
    "ALL",
)
PAIRINGS = ("Auto", "By index", "All combinations")


def find_matlab() -> Path | None:
    """Locate MATLAB for optional native FIG conversion."""
    candidates: list[Path] = []
    discovered = shutil.which("matlab")
    if discovered:
        candidates.append(Path(discovered))
    for base in (Path("/Applications"), Path.home() / "Applications"):
        if base.is_dir():
            candidates.extend(sorted(base.glob("MATLAB*.app/bin/matlab"), reverse=True))
    return next(
        (path.resolve() for path in candidates if path.is_file() and os.access(path, os.X_OK)),
        None,
    )


class FilePicker(ttk.LabelFrame):
    """List-based selector that accepts one or more sensor files."""

    def __init__(
        self,
        parent: tk.Misc,
        title: str,
        filetypes: tuple[tuple[str, str], ...],
        defaults: tuple[Path, ...] = (),
    ) -> None:
        super().__init__(parent, text=title, padding=6)
        self.filetypes = filetypes
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.listbox = tk.Listbox(self, height=4, selectmode=tk.EXTENDED)
        self.listbox.grid(row=0, column=0, columnspan=3, sticky="nsew")
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.listbox.yview)
        scrollbar.grid(row=0, column=3, sticky="ns")
        self.listbox.configure(yscrollcommand=scrollbar.set)
        ttk.Button(self, text="Add…", command=self.add).grid(row=1, column=0, sticky="w", pady=(5, 0))
        ttk.Button(self, text="Remove", command=self.remove).grid(row=1, column=1, pady=(5, 0))
        ttk.Button(self, text="Clear", command=self.clear).grid(row=1, column=2, sticky="e", pady=(5, 0))
        for path in defaults:
            if path.is_file():
                self.listbox.insert(tk.END, str(path))

    def add(self) -> None:
        selected = filedialog.askopenfilenames(
            initialdir=REPO_ROOT,
            filetypes=self.filetypes,
        )
        existing = set(self.paths())
        for value in selected:
            path = str(Path(value).expanduser().resolve())
            if path not in existing:
                self.listbox.insert(tk.END, path)
                existing.add(path)

    def remove(self) -> None:
        for index in reversed(self.listbox.curselection()):
            self.listbox.delete(index)

    def clear(self) -> None:
        self.listbox.delete(0, tk.END)

    def paths(self) -> list[str]:
        return list(self.listbox.get(0, tk.END))


class FusionGUI(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("GNSS/IMU Fusion — Tasks 1–7")
        self.geometry("1220x840")
        self.minsize(1000, 700)
        self.messages: queue.Queue[tuple[str, object]] = queue.Queue()
        self.process: subprocess.Popen[str] | None = None
        self.preview_image: tk.PhotoImage | None = None
        self.variables = {
            "output": tk.StringVar(value=str(REPO_ROOT / "results")),
            "config": tk.StringVar(value=""),
            "method": tk.StringVar(value="ALL"),
            "pairing": tk.StringVar(value="Auto"),
            "native_figs": tk.BooleanVar(value=True),
            "status": tk.StringVar(value="Ready"),
        }
        self.task_variables = {
            number: tk.BooleanVar(value=True) for number in range(1, 8)
        }
        self._build()
        self.after(100, self._drain_messages)

    def _build(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        controls = ttk.LabelFrame(self, text="Run configuration", padding=10)
        controls.grid(row=0, column=0, sticky="ew", padx=10, pady=8)
        for column in range(3):
            controls.columnconfigure(column, weight=1)

        self.imu_picker = FilePicker(
            controls,
            "IMU data (one or more)",
            (("IMU data", "*.dat *.txt *.csv"), ("All files", "*")),
            (REPO_ROOT / "DATA/IMU/IMU_X001_small.dat",),
        )
        self.gnss_picker = FilePicker(
            controls,
            "GNSS data (one or more)",
            (("GNSS data", "*.csv *.txt"), ("All files", "*")),
            (REPO_ROOT / "DATA/GNSS/GNSS_X001_small.csv",),
        )
        self.truth_picker = FilePicker(
            controls,
            "Truth data (optional, one or more)",
            (("Truth data", "*.txt *.dat *.csv"), ("All files", "*")),
            (REPO_ROOT / "DATA/Truth/STATE_X001_small.txt",),
        )
        self.imu_picker.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        self.gnss_picker.grid(row=0, column=1, sticky="nsew", padx=5)
        self.truth_picker.grid(row=0, column=2, sticky="nsew", padx=(5, 0))

        options = ttk.Frame(controls)
        options.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(9, 0))
        options.columnconfigure(7, weight=1)
        ttk.Label(options, text="Pairing").grid(row=0, column=0, padx=(0, 4))
        ttk.Combobox(
            options,
            textvariable=self.variables["pairing"],
            values=PAIRINGS,
            state="readonly",
            width=17,
        ).grid(row=0, column=1, padx=(0, 12))
        ttk.Label(options, text="Method").grid(row=0, column=2, padx=(0, 4))
        ttk.Combobox(
            options,
            textvariable=self.variables["method"],
            values=METHODS,
            width=20,
        ).grid(row=0, column=3, padx=(0, 12))
        ttk.Checkbutton(
            options,
            text="Create native MATLAB FIG",
            variable=self.variables["native_figs"],
        ).grid(row=0, column=4, padx=(0, 12))
        ttk.Label(options, text="Output").grid(row=0, column=5, padx=(0, 4))
        ttk.Entry(options, textvariable=self.variables["output"]).grid(row=0, column=6, sticky="ew")
        ttk.Button(options, text="Browse…", command=self._choose_output).grid(row=0, column=7, sticky="w", padx=4)

        task_row = ttk.Frame(controls)
        task_row.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        ttk.Label(task_row, text="Tasks").pack(side="left")
        for number in range(1, 8):
            ttk.Checkbutton(
                task_row,
                text=str(number),
                variable=self.task_variables[number],
            ).pack(side="left", padx=3)
        ttk.Label(task_row, text="(prerequisites run automatically)").pack(side="left", padx=(5, 15))
        ttk.Label(task_row, text="Input/layout config").pack(side="left")
        ttk.Entry(task_row, textvariable=self.variables["config"], width=38).pack(side="left", padx=5, fill="x", expand=True)
        ttk.Button(task_row, text="Browse…", command=self._choose_config).pack(side="left")

        buttons = ttk.Frame(controls)
        buttons.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(9, 0))
        self.validate_button = ttk.Button(buttons, text="Validate inputs", command=lambda: self._start(True))
        self.validate_button.pack(side="left")
        self.run_button = ttk.Button(buttons, text="Run selected tasks", command=lambda: self._start(False))
        self.run_button.pack(side="left", padx=8)
        ttk.Button(buttons, text="Accepted input structure", command=self._show_contract).pack(side="left")
        ttk.Label(buttons, textvariable=self.variables["status"]).pack(side="right")

        notebook = ttk.Notebook(self)
        notebook.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0, 10))
        log_frame = ttk.Frame(notebook)
        result_frame = ttk.Panedwindow(notebook, orient=tk.VERTICAL)
        notebook.add(log_frame, text="Run log")
        notebook.add(result_frame, text="Generated figures and artifacts")

        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = tk.Text(log_frame, wrap="word", font=("TkFixedFont", 10))
        self.log.grid(row=0, column=0, sticky="nsew")
        log_scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        log_scrollbar.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=log_scrollbar.set)

        table_frame = ttk.Frame(result_frame)
        preview_frame = ttk.Frame(result_frame)
        result_frame.add(table_frame, weight=2)
        result_frame.add(preview_frame, weight=3)
        table_frame.columnconfigure(0, weight=1)
        table_frame.rowconfigure(0, weight=1)
        self.results = ttk.Treeview(table_frame, columns=("type", "path"), show="headings")
        self.results.heading("type", text="Type")
        self.results.heading("path", text="Path")
        self.results.column("type", width=75, stretch=False)
        self.results.column("path", width=980)
        self.results.grid(row=0, column=0, sticky="nsew")
        self.results.bind("<<TreeviewSelect>>", self._preview_selected)
        self.results.bind("<Double-1>", self._open_selected)
        ttk.Button(table_frame, text="Open selected", command=self._open_selected).grid(row=1, column=0, sticky="w", pady=5)
        preview_frame.columnconfigure(0, weight=1)
        preview_frame.rowconfigure(0, weight=1)
        self.preview = ttk.Label(preview_frame, text="Select a PNG figure to preview", anchor="center")
        self.preview.grid(row=0, column=0, sticky="nsew")

    def _choose_output(self) -> None:
        selected = filedialog.askdirectory(initialdir=REPO_ROOT)
        if selected:
            self.variables["output"].set(selected)

    def _choose_config(self) -> None:
        selected = filedialog.askopenfilename(
            initialdir=REPO_ROOT / "config",
            filetypes=(("Configuration", "*.yaml *.yml *.json"), ("All files", "*")),
        )
        if selected:
            self.variables["config"].set(selected)

    @staticmethod
    def _indexed_pairs(imus: list[str], gnss: list[str]) -> list[tuple[str, str]]:
        if len(imus) == len(gnss):
            return list(zip(imus, gnss))
        if len(imus) == 1:
            return [(imus[0], path) for path in gnss]
        if len(gnss) == 1:
            return [(path, gnss[0]) for path in imus]
        raise ValueError("By-index pairing requires equal counts or one reusable IMU/GNSS file.")

    def _runs(self) -> list[tuple[str, str, str | None]]:
        imus = self.imu_picker.paths()
        gnss = self.gnss_picker.paths()
        truths = self.truth_picker.paths()
        if not imus or not gnss:
            raise ValueError("Select at least one IMU file and one GNSS file.")
        pairing = self.variables["pairing"].get()
        if pairing == "All combinations":
            pairs = list(itertools.product(imus, gnss))
        elif pairing == "By index":
            pairs = self._indexed_pairs(imus, gnss)
        else:
            try:
                pairs = self._indexed_pairs(imus, gnss)
            except ValueError:
                pairs = list(itertools.product(imus, gnss))
        if not truths:
            return [(imu, gnss_path, None) for imu, gnss_path in pairs]
        if len(truths) == 1:
            return [(imu, gnss_path, truths[0]) for imu, gnss_path in pairs]
        if len(truths) != len(pairs):
            raise ValueError(
                "Truth count must be one or match the number of generated IMU/GNSS pairs."
            )
        return [(*pair, truth) for pair, truth in zip(pairs, truths)]

    def _commands(self, validate_only: bool) -> list[list[str]]:
        selected_tasks = [str(number) for number, value in self.task_variables.items() if value.get()]
        if not selected_tasks:
            raise ValueError("Select at least one task.")
        output = Path(self.variables["output"].get()).expanduser().resolve()
        config = self.variables["config"].get().strip()
        commands: list[list[str]] = []
        for imu, gnss, truth in self._runs():
            run_id = f"{Path(imu).stem}__{Path(gnss).stem}"
            if truth and len(self.truth_picker.paths()) > 1:
                run_id += f"__{Path(truth).stem}"
            command = [
                sys.executable,
                "-u",
                str(RUNNER),
                "--imu",
                imu,
                "--gnss",
                gnss,
                "--method",
                self.variables["method"].get(),
                "--tasks",
                ",".join(selected_tasks),
                "--output",
                str(output),
                "--run-id",
                run_id,
            ]
            if truth:
                command.extend(("--truth", truth))
            if config:
                command.extend(("--config", config))
            if validate_only:
                command.append("--validate-only")
            commands.append(command)
            if not validate_only and self.variables["native_figs"].get():
                commands.append(
                    [
                        sys.executable,
                        "-u",
                        str(RELEASE_RUNNER),
                        "--export-figs-only",
                        "--output",
                        str(output / run_id),
                    ]
                )
        return commands

    def _preflight(self) -> bool:
        try:
            runs = self._runs()
            for path in itertools.chain.from_iterable(
                (imu, gnss, *((truth,) if truth else ())) for imu, gnss, truth in runs
            ):
                if not Path(path).expanduser().is_file():
                    raise ValueError(f"Input file does not exist: {path}")
            config = self.variables["config"].get().strip()
            if config and not Path(config).expanduser().is_file():
                raise ValueError(f"Configuration file does not exist: {config}")
            if self.variables["native_figs"].get() and find_matlab() is None:
                self.variables["native_figs"].set(False)
                messagebox.showwarning(
                    "MATLAB not found",
                    "PNG, PDF, and MAT files will still be created. Native FIG "
                    "conversion was disabled because no MATLAB executable was found.",
                )
            return True
        except ValueError as exc:
            messagebox.showerror("Invalid run configuration", str(exc))
            return False

    def _start(self, validate_only: bool) -> None:
        if self.process is not None or not self._preflight():
            return
        try:
            commands = self._commands(validate_only)
        except ValueError as exc:
            messagebox.showerror("Invalid run configuration", str(exc))
            return
        self.log.delete("1.0", tk.END)
        self.results.delete(*self.results.get_children())
        self.variables["status"].set("Validating…" if validate_only else "Running…")
        self.run_button.configure(state="disabled")
        self.validate_button.configure(state="disabled")
        threading.Thread(target=self._worker, args=(commands,), daemon=True).start()

    def _worker(self, commands: list[list[str]]) -> None:
        code = 0
        try:
            for index, command in enumerate(commands, 1):
                self.messages.put(("log", f"\n[{index}/{len(commands)}] $ {' '.join(command)}\n\n"))
                self.process = subprocess.Popen(
                    command,
                    cwd=REPO_ROOT,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
                assert self.process.stdout is not None
                for line in self.process.stdout:
                    self.messages.put(("log", line))
                code = self.process.wait()
                if code:
                    break
        except Exception as exc:
            self.messages.put(("log", f"GUI launch error: {exc}\n"))
            code = 1
        self.messages.put(("done", code))

    def _drain_messages(self) -> None:
        try:
            while True:
                kind, value = self.messages.get_nowait()
                if kind == "log":
                    self.log.insert(tk.END, str(value))
                    self.log.see(tk.END)
                elif kind == "done":
                    self.process = None
                    code = int(value)
                    self.variables["status"].set("Complete" if code == 0 else f"Failed (exit {code})")
                    self.run_button.configure(state="normal")
                    self.validate_button.configure(state="normal")
                    self._refresh_results()
        except queue.Empty:
            pass
        self.after(100, self._drain_messages)

    def _refresh_results(self) -> None:
        root = Path(self.variables["output"].get()).expanduser()
        if not root.is_dir():
            return
        suffixes = {".png", ".pdf", ".fig", ".mat", ".json", ".csv", ".npz"}
        files = sorted(path for path in root.rglob("*") if path.suffix.lower() in suffixes)
        for path in files:
            self.results.insert("", "end", values=(path.suffix[1:].upper(), str(path.resolve())))

    def _selected_path(self) -> Path | None:
        selected = self.results.selection()
        if not selected:
            return None
        return Path(self.results.item(selected[0], "values")[1])

    def _preview_selected(self, _event=None) -> None:
        path = self._selected_path()
        if path is None or path.suffix.lower() != ".png":
            self.preview.configure(image="", text="Select a PNG figure to preview")
            self.preview_image = None
            return
        try:
            image = tk.PhotoImage(file=str(path))
            factor = max(1, math.ceil(max(image.width() / 1000, image.height() / 470)))
            if factor > 1:
                image = image.subsample(factor, factor)
            self.preview_image = image
            self.preview.configure(image=image, text="")
        except tk.TclError:
            self.preview.configure(image="", text=f"Preview unavailable\n{path}")

    def _open_selected(self, _event=None) -> None:
        path = self._selected_path()
        if path is None:
            return
        if sys.platform == "darwin":
            subprocess.Popen(["open", str(path)])
        elif os.name == "nt":
            os.startfile(path)  # type: ignore[attr-defined]
        else:
            subprocess.Popen(["xdg-open", str(path)])

    def _show_contract(self) -> None:
        result = subprocess.run(
            [sys.executable, str(RUNNER), "--print-contract"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
        messagebox.showinfo("Accepted input structure", result.stdout or result.stderr)


def main() -> None:
    FusionGUI().mainloop()


if __name__ == "__main__":
    main()
