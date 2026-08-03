#!/usr/bin/env python3
"""Tkinter front-end for the canonical Task 1-7 fusion pipeline."""

from __future__ import annotations

import os
import queue
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

REPO_ROOT = Path(__file__).resolve().parent
RUNNER = REPO_ROOT / "PYTHON" / "run_pipeline.py"
# Any comma-separated subset is also accepted, so the box stays editable.
METHODS = (
    "TRIAD",
    "Davenport",
    "SVD",
    "TRIAD,Davenport",
    "TRIAD,SVD",
    "Davenport,SVD",
    "ALL",
)


class FusionGUI(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("GNSS/IMU Fusion — Tasks 1–7")
        self.geometry("1050x720")
        self.minsize(850, 600)
        self.messages: queue.Queue[tuple[str, object]] = queue.Queue()
        self.process: subprocess.Popen[str] | None = None
        self.variables = {
            "imu": tk.StringVar(value=str(REPO_ROOT / "DATA" / "IMU" / "IMU_X001_small.dat")),
            "gnss": tk.StringVar(value=str(REPO_ROOT / "DATA" / "GNSS" / "GNSS_X001_small.csv")),
            "truth": tk.StringVar(value=str(REPO_ROOT / "DATA" / "Truth" / "STATE_X001_small.txt")),
            "output": tk.StringVar(value=str(REPO_ROOT / "results")),
            "method": tk.StringVar(value="ALL"),
            "tasks": tk.StringVar(value="1-7"),
            "status": tk.StringVar(value="Ready"),
        }
        self._build()
        self.after(100, self._drain_messages)

    def _build(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        controls = ttk.LabelFrame(self, text="Run configuration", padding=12)
        controls.grid(row=0, column=0, sticky="ew", padx=10, pady=8)
        controls.columnconfigure(1, weight=1)

        self._file_row(controls, 0, "IMU input", "imu", (("IMU data", "*.dat *.txt"), ("All files", "*")))
        self._file_row(controls, 1, "GNSS input", "gnss", (("GNSS CSV", "*.csv"), ("All files", "*")))
        self._file_row(controls, 2, "Truth (optional)", "truth", (("Truth data", "*.txt *.dat"), ("All files", "*")))
        self._file_row(controls, 3, "Output folder", "output", None)

        ttk.Label(controls, text="Method").grid(row=4, column=0, sticky="w", pady=4)
        ttk.Combobox(controls, textvariable=self.variables["method"], values=METHODS, width=18).grid(row=4, column=1, sticky="w", pady=4)
        ttk.Label(controls, text="Tasks").grid(row=4, column=1, sticky="e", padx=(0, 230))
        ttk.Entry(controls, textvariable=self.variables["tasks"], width=16).grid(row=4, column=1, sticky="e", padx=(0, 70))
        ttk.Label(controls, text="Examples: 3, 1-5, 1-7").grid(row=4, column=2, sticky="w")

        buttons = ttk.Frame(controls)
        buttons.grid(row=5, column=0, columnspan=3, sticky="ew", pady=(10, 0))
        self.validate_button = ttk.Button(buttons, text="Validate inputs", command=lambda: self._start(validate_only=True))
        self.validate_button.pack(side="left")
        self.run_button = ttk.Button(buttons, text="Run selected tasks", command=lambda: self._start(validate_only=False))
        self.run_button.pack(side="left", padx=8)
        ttk.Button(buttons, text="Input contract", command=self._show_contract).pack(side="left")
        ttk.Label(buttons, textvariable=self.variables["status"]).pack(side="right")

        notebook = ttk.Notebook(self)
        notebook.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0, 10))
        log_frame = ttk.Frame(notebook)
        results_frame = ttk.Frame(notebook)
        notebook.add(log_frame, text="Run log")
        notebook.add(results_frame, text="Generated plots and artifacts")

        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = tk.Text(log_frame, wrap="word", font=("TkFixedFont", 10))
        self.log.grid(row=0, column=0, sticky="nsew")
        log_scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        log_scrollbar.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=log_scrollbar.set)

        results_frame.columnconfigure(0, weight=1)
        results_frame.rowconfigure(0, weight=1)
        self.results = ttk.Treeview(results_frame, columns=("type", "path"), show="headings")
        self.results.heading("type", text="Type")
        self.results.heading("path", text="Path")
        self.results.column("type", width=100, stretch=False)
        self.results.column("path", width=760)
        self.results.grid(row=0, column=0, sticky="nsew")
        self.results.bind("<Double-1>", self._open_selected)
        ttk.Button(results_frame, text="Open selected", command=self._open_selected).grid(row=1, column=0, sticky="w", pady=6)

    def _file_row(self, parent: ttk.LabelFrame, row: int, label: str, key: str, filetypes) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(parent, textvariable=self.variables[key]).grid(row=row, column=1, sticky="ew", padx=8, pady=4)
        command = (lambda: self._choose_directory(key)) if filetypes is None else (lambda: self._choose_file(key, filetypes))
        ttk.Button(parent, text="Browse…", command=command).grid(row=row, column=2, pady=4)

    def _choose_file(self, key: str, filetypes) -> None:
        selected = filedialog.askopenfilename(initialdir=REPO_ROOT, filetypes=filetypes)
        if selected:
            self.variables[key].set(selected)

    def _choose_directory(self, key: str) -> None:
        selected = filedialog.askdirectory(initialdir=REPO_ROOT)
        if selected:
            self.variables[key].set(selected)

    def _command(self, validate_only: bool) -> list[str]:
        command = [
            sys.executable,
            "-u",
            str(RUNNER),
            "--imu",
            self.variables["imu"].get().strip(),
            "--gnss",
            self.variables["gnss"].get().strip(),
            "--method",
            self.variables["method"].get(),
            "--tasks",
            self.variables["tasks"].get().strip(),
            "--output",
            self.variables["output"].get().strip(),
        ]
        truth = self.variables["truth"].get().strip()
        if truth:
            command.extend(("--truth", truth))
        if validate_only:
            command.append("--validate-only")
        return command

    def _preflight(self) -> bool:
        for key, label in (("imu", "IMU"), ("gnss", "GNSS")):
            path = Path(self.variables[key].get().strip()).expanduser()
            if not path.is_file():
                messagebox.showerror("Missing input", f"{label} file does not exist:\n{path}")
                return False
        truth = self.variables["truth"].get().strip()
        if truth and not Path(truth).expanduser().is_file():
            messagebox.showerror("Missing input", f"Truth file does not exist:\n{truth}")
            return False
        if not self.variables["tasks"].get().strip():
            messagebox.showerror("Invalid tasks", "Enter a task selection such as 3, 1-5 or 1-7.")
            return False
        return True

    def _start(self, validate_only: bool) -> None:
        if self.process is not None or not self._preflight():
            return
        self.log.delete("1.0", tk.END)
        self.results.delete(*self.results.get_children())
        self.variables["status"].set("Validating…" if validate_only else "Running…")
        self.run_button.configure(state="disabled")
        self.validate_button.configure(state="disabled")
        command = self._command(validate_only)
        self._append("$ " + " ".join(command) + "\n\n")
        threading.Thread(target=self._worker, args=(command,), daemon=True).start()

    def _worker(self, command: list[str]) -> None:
        try:
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
            self.messages.put(("done", code))
        except Exception as exc:
            self.messages.put(("log", f"GUI launch error: {exc}\n"))
            self.messages.put(("done", 1))

    def _drain_messages(self) -> None:
        try:
            while True:
                kind, value = self.messages.get_nowait()
                if kind == "log":
                    self._append(str(value))
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

    def _append(self, text: str) -> None:
        self.log.insert(tk.END, text)
        self.log.see(tk.END)

    def _refresh_results(self) -> None:
        root = Path(self.variables["output"].get().strip()).expanduser()
        if not root.is_dir():
            return
        patterns = ("*.png", "*.json", "*.csv", "*.npz")
        files = sorted(path for pattern in patterns for path in root.rglob(pattern))
        for path in files:
            self.results.insert("", "end", values=(path.suffix.lstrip(".").upper(), str(path.resolve())))

    def _open_selected(self, _event=None) -> None:
        selected = self.results.selection()
        if not selected:
            return
        path = self.results.item(selected[0], "values")[1]
        if sys.platform == "darwin":
            subprocess.Popen(["open", path])
        elif os.name == "nt":
            os.startfile(path)  # type: ignore[attr-defined]
        else:
            subprocess.Popen(["xdg-open", path])

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
