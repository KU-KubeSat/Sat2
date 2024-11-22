# KUbesat systems

The code for the Kubesat 2 systems

## Contributing

We use the [Gitflow workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow/). This means we have a `dev` branch and a `main` branch.

**Main Branch:**

- Purpose: The stable, production-ready code that will be deployed to the satellite.
- Access: Direct changes to this branch are strictly prohibited. All changes must be merged through pull requests from the dev branch.

**Dev Branch:**

- Purpose: The active development branch where ongoing features and bug fixes are implemented.
- Branching: Create a personal branch off of dev to work on your specific changes. Use the format initials-feature-name (e.g., vp-add-new-feature).
- Committing: Commit your changes to your personal branch regularly.
- Pushing: Push your branch to the remote repository.

**Pull Requests:**

- Creating: Once your feature is complete, create a pull request from your personal branch to the dev branch.
- Description: Provide a clear and concise description of your changes in the pull request.
- Review: Your pull request will be reviewed by the team.
- Merging: Once approved, your changes will be merged into the dev branch.

**Remember:**

- Always work on a dedicated branch for each feature or bug fix.
- Commit your changes frequently and write clear commit messages.
- Review your code before pushing it.
- Use the pull request process to ensure code quality and collaboration.
