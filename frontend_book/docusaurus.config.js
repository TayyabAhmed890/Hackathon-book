// @ts-check
// `@type` JSDoc annotations allow IDEs and type-checking tools to autocomplete
// and validate on `docusaurus.config.js` APIs.

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Complete Guide to Robotics with ROS 2, Gazebo, and Unity',
  favicon: '/img/favicon.ico',

  // Set the production url of your site here
  url: 'https://', // Vercel will handle the domain automatically
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',
  // Additional settings for Vercel deployment
  trailingSlash: false,

  // GitHub pages deployment config.
  // If you aren't using GitHub Pages, you don't need these.
  organizationName: 'TayyabAhmed890', // Usually your GitHub org/user name.
  projectName: 'Hackathon-book', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/TayyabAhmed890/Hackathon-book/edit/main/frontend_book/',
        },
        blog: false, // Optional: disable the blog plugin
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      navbar: {
        title: 'Physical AI & Robotics',
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/facebook/docusaurus',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Book',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'Module 1: ROS 2 Fundamentals',
                to: '/docs/module-1/intro',
              },
              {
                label: 'Module 2: Digital Twin Simulation',
                to: '/docs/module-2/intro',
              },
              {
                label: 'Module 3: NVIDIA Isaac AI',
                to: '/docs/module-3/intro',
              },
              {
                label: 'Module 4: Vision-Language-Action (VLA)',
                to: '/docs/module-4-vla/index',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/ros+or+robotics',
              },
              {
                label: 'Robotics Forum',
                href: 'https://discourse.ros.org/',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/TayyabAhmed890/Hackathon-book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;