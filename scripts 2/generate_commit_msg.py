#!/usr/bin/env python3
"""
AI Commit Message Generator
===========================

Generates a commit message based on staged changes using OpenAI's API.
"""

import os
import subprocess
import sys
from typing import Optional

import typer
from rich.console import Console
from rich.panel import Panel
from rich.prompt import Confirm, Prompt

try:
    from openai import OpenAI, OpenAIError
except ImportError:
    print("Error: 'openai' package not found. Please install it with: pip install openai")
    sys.exit(1)

app = typer.Typer(help="AI Commit Message Generator")
console = Console()


def get_staged_diff() -> str:
    """Get the diff of staged changes."""
    result = subprocess.run(
        ["git", "diff", "--cached"], capture_output=True, text=True, check=False
    )
    if result.returncode != 0:
        console.print("[bold red]Error running git diff.[/bold red]")
        sys.exit(1)

    return result.stdout.strip()


def generate_message(diff: str, api_key: str, model: str) -> str:
    """Generate a commit message using OpenAI."""
    client = OpenAI(api_key=api_key)

    system_prompt = (
        "You are an expert software engineer. "
        "Generate a concise, conventional commit message for the following code changes. "
        "The message should have a summary line (max 50 chars) and a body (wrapped at 72 chars). "
        "Use the format:\n"
        "<type>(<scope>): <subject>\n\n"
        "<body>"
    )

    try:
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Here are the staged changes:\n\n{diff}"},
            ],
            temperature=0.7,
            max_tokens=200,
        )
        return response.choices[0].message.content.strip()
    except OpenAIError as e:
        console.print(f"[bold red]OpenAI API Error:[/bold red] {e}")
        sys.exit(1)


@app.command()
def main(
    model: str = typer.Option("gpt-3.5-turbo", "--model", "-m", help="OpenAI model to use"),
    dry_run: bool = typer.Option(False, "--dry-run", help="Generate message but do not commit"),
):
    """
    Generate a commit message from staged changes and optionally commit.
    """
    console.print(Panel("AI Commit Message Generator", style="bold blue"))

    # 1. Check for staged changes
    diff = get_staged_diff()
    if not diff:
        console.print(
            "[yellow]No staged changes found. Use 'git add' to stage files first.[/yellow]"
        )
        raise typer.Exit()

    # 2. Get API Key
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        api_key = Prompt.ask("Please enter your OpenAI API Key", password=True)
        if not api_key:
            console.print("[red]API Key is required.[/red]")
            raise typer.Exit(1)

    # 3. Generate Message
    with console.status("[bold green]Generating commit message...[/bold green]"):
        message = generate_message(diff, api_key, model)

    console.print("\n[bold]Generated Commit Message:[/bold]")
    console.print(Panel(message, title="Commit Message", border_style="green"))

    if dry_run:
        console.print("[dim]Dry run: Commit not executed.[/dim]")
        return

    # 4. Confirm and Commit
    if Confirm.ask("Do you want to use this commit message?"):
        try:
            subprocess.run(["git", "commit", "-m", message], check=True)
            console.print("[bold green]Commit successful![/bold green]")
        except subprocess.CalledProcessError:
            console.print("[bold red]Commit failed.[/bold red]")
            sys.exit(1)
    else:
        console.print("[yellow]Commit cancelled.[/yellow]")


if __name__ == "__main__":
    app()
