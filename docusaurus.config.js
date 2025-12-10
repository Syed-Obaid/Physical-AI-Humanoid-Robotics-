// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('@docusaurus/theme-classic').default;
const darkCodeTheme = require('@docusaurus/theme-classic').default;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'ROBOX',
  tagline: 'Physical AI & Humanoid Robotics',
  favicon: 'img/logo.svg',

  // Set the production url of your site here
  // url: 'https://Syed-Obaid.github.io',
  url: 'https://physical-ai-humanoid-robotics-opal.vercel.app/',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages, this is usually '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Syed-Obaid', // Usually your GitHub org/user name.
  projectName: 'docusaurus-book', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
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
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Syed-Obaid/',
          routeBasePath: 'docs', // Serve docs under /docs path
          showLastUpdateTime: true,
        },
        blog: false, // Disable blog for book format
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
        gtag: {
          trackingID: 'G-XXXXXXXXXX',
          anonymizeIP: true,
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      metadata: [
        { name: 'keywords', content: 'book, docusaurus, spec-driven, ai, documentation, writing, publishing, education, technical writing, automated publishing' },
        { name: 'author', content: 'AI/Spec-Driven Book Creation System' },
        { name: 'description', content: 'Complete guide to building, writing, and deploying books using Docusaurus with AI assistance' },
        { name: 'og:type', content: 'website' },
        { name: 'og:title', content: 'AI/Spec-Driven Book Creation System' },
        { name: 'og:description', content: 'Build, Write & Deploy Books Using Docusaurus' },
        { name: 'og:url', content: 'https://Syed-Obaid.github.io/' },
        { name: 'twitter:card', content: 'summary_large_image' },
        { name: 'twitter:title', content: 'AI/Spec-Driven Book Creation System' },
        { name: 'twitter:description', content: 'Build, Write & Deploy Books Using Docusaurus' },
      ],
      algolia: {
        // The application ID provided by Algolia
        appId: 'YOUR_APP_ID',
        // Public API key: it is safe to commit it
        apiKey: 'YOUR_SEARCH_API_KEY',
        indexName: 'your-book-index',
        // Optional: see doc link below
        contextualSearch: true,
        // Optional: Specify domains where the navigation should occur through window.location instead on history.push. Useful when our Algolia config crawls multiple documentation sites and we want to navigate with window.location.href to them.
        // externalUrlRegex: 'external\\.com|domain\\.com',
        // Optional: Replace parts of the item URLs from Algolia. Useful when using the same search index for multiple deployments using a different baseUrl. You can use regexp or string in the `from` param. For example: localhost:3000 vs myCompany.com/docs
        // replaceSearchResultPathname: {
        //   from: '/docs/', // or as regex: /\/docs\//
        //   to: '/',
        // },
        // Optional: Algolia search parameters
        // searchParameters: {},
        // Optional: path for search page that enabled by default (`false` to disable it)
        searchPagePath: 'search',
      },
      docs: {
        versionPersistence: 'localStorage',
      },
      navbar: {
        title: 'ROBOX',
        logo: {
          alt: 'ROBOX Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/Syed-Obaid/Physical-AI-Humanoid-Robotics-',
            label: 'GitHub',
            position: 'right',
          },
          {
            to: '/docs/',
            label: 'Get Started',
            position: 'right',
            className: 'navbar-cta-button',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Book',
                to: '/docs/module1/overview',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Syed-Obaid/',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} ROBOX Inc. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;